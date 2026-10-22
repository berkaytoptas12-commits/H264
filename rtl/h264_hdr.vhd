-- ============================================================================
-- h264_hdr.vhd
-- NAL dispatcher, SPS / PPS / slice header parser and decoded picture buffer
-- management:
--   * SPS (id 0..31) and PPS (id 0..255) tables in block RAM
--   * frame buffer allocation, reference marking (sliding window, MMCO 1..6,
--     long-term references), reference list initialisation and
--     ref_pic_list_modification for P slices
--   * hands every slice to the macroblock parser (h264_mbparse)
--   * frame output in decoding order once the deblocking stage reports that
--     a picture is completely written; buffers return on i_release
-- Unsupported streams (CABAC, FMO, B/SP/SI slices, interlace, 8x8 transform,
-- scaling matrices, weighted prediction, lossless) set o_error and their
-- slices are skipped.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;

entity h264_hdr is
    port (
        clk           : in  std_logic;
        rst           : in  std_logic;
        -- bit reader
        br_win        : in  std_logic_vector(63 downto 0);
        br_avail64    : in  std_logic;
        br_more       : in  std_logic;
        br_active     : in  std_logic;
        br_eos        : in  std_logic;      -- end of stream seen by the bit reader
        hdr_consume   : out unsigned(6 downto 0);
        br_flush      : out std_logic;
        own_br        : out std_logic;
        -- macroblock parser
        slice         : out slice_t;
        slice_start   : out std_logic;
        slice_done    : in  std_logic;
        -- deblocking stage: picture completely written
        pic_done      : in  std_logic;
        pic_done_fb   : in  unsigned(4 downto 0);
        -- decoded frame output (decoding order)
        o_frame_valid : out std_logic;
        o_frame_fb    : out unsigned(4 downto 0);
        i_frame_ready : in  std_logic;
        i_release     : in  std_logic;
        i_release_fb  : in  unsigned(4 downto 0);
        -- picture geometry of the active SPS
        o_mb_w        : out unsigned(7 downto 0);
        o_mb_h        : out unsigned(7 downto 0);
        o_crop_l      : out unsigned(9 downto 0);
        o_crop_r      : out unsigned(9 downto 0);
        o_crop_t      : out unsigned(9 downto 0);
        o_crop_b      : out unsigned(9 downto 0);
        -- status
        o_error       : out std_logic;
        o_stream_done : out std_logic;
        o_pic_count   : out unsigned(31 downto 0)
    );
end entity;

architecture rtl of h264_hdr is

    constant FB_NONE : unsigned(4 downto 0) := to_unsigned(31, 5);

    -- ---------------------------------------------------------------
    -- parameter set storage
    -- ---------------------------------------------------------------
    type sps_t is record
        valid    : std_logic;
        unsup    : std_logic;
        log2fn   : unsigned(4 downto 0);
        poc_type : unsigned(1 downto 0);
        log2poc  : unsigned(4 downto 0);
        dpoaz    : std_logic;
        nrf      : unsigned(4 downto 0);
        mb_w     : unsigned(7 downto 0);
        mb_h     : unsigned(7 downto 0);
        crop_l   : unsigned(9 downto 0);
        crop_r   : unsigned(9 downto 0);
        crop_t   : unsigned(9 downto 0);
        crop_b   : unsigned(9 downto 0);
    end record;
    constant SPS_BITS : natural := 1 + 1 + 5 + 2 + 5 + 1 + 5 + 8 + 8 + 40;
    constant SPS_INIT : sps_t := ('0', '0', (others => '0'), (others => '0'), (others => '0'), '0',
                                  (others => '0'), (others => '0'), (others => '0'),
                                  (others => '0'), (others => '0'), (others => '0'), (others => '0'));

    type pps_t is record
        valid    : std_logic;
        unsup    : std_logic;
        sps_id   : unsigned(4 downto 0);
        bfpo     : std_logic;
        nref     : unsigned(5 downto 0);
        init_qp  : unsigned(5 downto 0);
        cqp_off  : signed(4 downto 0);
        dbctl    : std_logic;
        cip      : std_logic;
        rpc      : std_logic;
    end record;
    constant PPS_BITS : natural := 1 + 1 + 5 + 1 + 6 + 6 + 5 + 1 + 1 + 1;
    constant PPS_INIT : pps_t := ('0', '0', (others => '0'), '0', (others => '0'), (others => '0'),
                                  (others => '0'), '0', '0', '0');

    function pack_sps(s : sps_t) return std_logic_vector is
    begin
        return s.valid & s.unsup & std_logic_vector(s.log2fn) & std_logic_vector(s.poc_type) &
               std_logic_vector(s.log2poc) & s.dpoaz & std_logic_vector(s.nrf) &
               std_logic_vector(s.mb_w) & std_logic_vector(s.mb_h) & std_logic_vector(s.crop_l) &
               std_logic_vector(s.crop_r) & std_logic_vector(s.crop_t) & std_logic_vector(s.crop_b);
    end function;

    function unpack_sps(v : std_logic_vector(SPS_BITS - 1 downto 0)) return sps_t is
        variable s : sps_t;
    begin
        s.valid    := v(75);
        s.unsup    := v(74);
        s.log2fn   := unsigned(v(73 downto 69));
        s.poc_type := unsigned(v(68 downto 67));
        s.log2poc  := unsigned(v(66 downto 62));
        s.dpoaz    := v(61);
        s.nrf      := unsigned(v(60 downto 56));
        s.mb_w     := unsigned(v(55 downto 48));
        s.mb_h     := unsigned(v(47 downto 40));
        s.crop_l   := unsigned(v(39 downto 30));
        s.crop_r   := unsigned(v(29 downto 20));
        s.crop_t   := unsigned(v(19 downto 10));
        s.crop_b   := unsigned(v(9 downto 0));
        return s;
    end function;

    function pack_pps(p : pps_t) return std_logic_vector is
    begin
        return p.valid & p.unsup & std_logic_vector(p.sps_id) & p.bfpo & std_logic_vector(p.nref) &
               std_logic_vector(p.init_qp) & std_logic_vector(p.cqp_off) & p.dbctl & p.cip & p.rpc;
    end function;

    function unpack_pps(v : std_logic_vector(PPS_BITS - 1 downto 0)) return pps_t is
        variable p : pps_t;
    begin
        p.valid   := v(27);
        p.unsup   := v(26);
        p.sps_id  := unsigned(v(25 downto 21));
        p.bfpo    := v(20);
        p.nref    := unsigned(v(19 downto 14));
        p.init_qp := unsigned(v(13 downto 8));
        p.cqp_off := signed(v(7 downto 3));
        p.dbctl   := v(2);
        p.cip     := v(1);
        p.rpc     := v(0);
        return p;
    end function;

    type sps_ram_t is array (0 to 31) of std_logic_vector(SPS_BITS - 1 downto 0);
    type pps_ram_t is array (0 to 255) of std_logic_vector(PPS_BITS - 1 downto 0);
    signal sps_ram : sps_ram_t;
    signal pps_ram : pps_ram_t;
    signal sps_we, pps_we : std_logic;
    signal sps_wa : unsigned(4 downto 0);
    signal pps_wa : unsigned(7 downto 0);
    signal sps_wd : std_logic_vector(SPS_BITS - 1 downto 0);
    signal pps_wd : std_logic_vector(PPS_BITS - 1 downto 0);
    signal sps_ra : unsigned(4 downto 0) := (others => '0');
    signal pps_ra : unsigned(7 downto 0) := (others => '0');
    signal sps_rd : std_logic_vector(SPS_BITS - 1 downto 0);
    signal pps_rd : std_logic_vector(PPS_BITS - 1 downto 0);

    -- ---------------------------------------------------------------
    -- decoded picture buffer
    -- ---------------------------------------------------------------
    constant REF_NONE  : unsigned(1 downto 0) := "00";
    constant REF_SHORT : unsigned(1 downto 0) := "01";
    constant REF_LONG  : unsigned(1 downto 0) := "10";
    type ref_arr_t is array (0 to C_NUM_FB - 1) of unsigned(1 downto 0);
    type fn_arr_t  is array (0 to C_NUM_FB - 1) of unsigned(15 downto 0);
    type lt_arr_t  is array (0 to C_NUM_FB - 1) of unsigned(4 downto 0);
    type pn_arr_t  is array (0 to C_NUM_FB - 1) of signed(17 downto 0);
    signal dpb_ref  : ref_arr_t;
    signal dpb_fn   : fn_arr_t;
    signal dpb_lt   : lt_arr_t;
    signal dpb_disp : std_logic_vector(C_NUM_FB - 1 downto 0);   -- held by display
    signal dpb_outp : std_logic_vector(C_NUM_FB - 1 downto 0);   -- waiting for output
    signal dpb_wr   : std_logic_vector(C_NUM_FB - 1 downto 0);   -- completely written
    signal max_lt   : signed(5 downto 0);

    type rl_t is array (0 to 16) of unsigned(4 downto 0);
    signal rl       : rl_t;
    signal rl_len   : unsigned(4 downto 0);
    signal rl_taken : std_logic_vector(C_NUM_FB - 1 downto 0);
    signal rl_idx   : unsigned(4 downto 0);     -- refIdxLX in modification
    signal rl_pred  : signed(17 downto 0);      -- picNumLXPred
    signal rl_tgt   : unsigned(4 downto 0);
    signal rl_c     : unsigned(4 downto 0);
    signal rl_n     : unsigned(4 downto 0);
    signal mod_idc  : unsigned(1 downto 0);

    -- output queue (decoding order)
    type oq_t is array (0 to 3) of unsigned(4 downto 0);
    signal oq       : oq_t;
    signal oq_cnt   : unsigned(2 downto 0);

    -- ---------------------------------------------------------------
    -- parser state
    -- ---------------------------------------------------------------
    type state_t is (
        S_IDLE, S_NALHDR, S_END_NAL, S_WAIT_FLUSH,
        -- SPS
        S_SPS_PROF, S_SPS_CONS, S_SPS_LEVEL, S_SPS_ID, S_SPS_CF, S_SPS_BDY, S_SPS_BDC, S_SPS_BYP,
        S_SPS_SCAL, S_SPS_LOG2FN, S_SPS_POCT, S_SPS_LOG2POC, S_SPS_DPOAZ, S_SPS_ONRP, S_SPS_OTTB,
        S_SPS_NCYC, S_SPS_OFS, S_SPS_NRF, S_SPS_GAPS, S_SPS_W, S_SPS_H, S_SPS_FMO, S_SPS_D8,
        S_SPS_CROP, S_SPS_CL, S_SPS_CR, S_SPS_CT, S_SPS_CB, S_SPS_STORE,
        -- PPS
        S_PPS_ID, S_PPS_SPSID, S_PPS_ENT, S_PPS_BFPO, S_PPS_NSG, S_PPS_NREF0, S_PPS_NREF1,
        S_PPS_WP, S_PPS_WB, S_PPS_QP, S_PPS_QS, S_PPS_CQP, S_PPS_DBC, S_PPS_CIP, S_PPS_RPC,
        S_PPS_T8, S_PPS_PSM, S_PPS_STORE,
        -- slice header
        S_SL_FIRST, S_SL_TYPE, S_SL_PPS, S_SL_RDPPS, S_SL_RDSPS, S_SL_RDSPS2, S_SL_CHECK, S_SL_FN,
        S_PIC_FINISH, S_MK_MMCO, S_MK_OPSCAN, S_MK_SLIDE, S_MK_CUR, S_PIC_ALLOC,
        S_SL_IDR, S_SL_POCLSB, S_SL_DPOCB, S_SL_DPOC0, S_SL_DPOC1, S_SL_RPC,
        S_SL_NROVR, S_SL_NREF, S_RL_INIT, S_RL_SHORT, S_RL_LONG,
        S_SL_RPLM, S_SL_MODIDC, S_SL_MODVAL, S_RL_MODFIND, S_RL_MODSHIFT, S_RL_COMPACT,
        S_SL_NOOUT, S_SL_LTREF, S_SL_ADAPT, S_SL_MMCO, S_SL_MMCO_A, S_SL_MMCO_B,
        S_SL_QPD, S_SL_DDF, S_SL_ALPHA, S_SL_BETA, S_SL_START, S_SL_WAIT,
        S_EOS_FINISH, S_DONE);
    signal st : state_t;
    signal st_dbg : natural;   -- simulation aid
    signal ret_st : state_t;

    signal nal_type : unsigned(4 downto 0);
    signal nal_ref  : unsigned(1 downto 0);
    signal t_sps    : sps_t;
    signal t_id     : unsigned(7 downto 0);
    signal t_pps    : pps_t;
    signal hi_prof  : std_logic;
    signal cnt      : unsigned(8 downto 0);
    signal act_sps  : sps_t;
    signal act_pps  : pps_t;
    signal err      : std_logic;

    -- slice header fields
    signal sl       : slice_t;
    signal sl_type_p : std_logic;
    signal sl_fn    : unsigned(15 downto 0);
    signal sl_idr   : std_logic;
    signal sl_first : unsigned(13 downto 0);
    signal sl_nref  : unsigned(5 downto 0);
    signal sl_skip  : std_logic;      -- slice not decodable

    -- current picture
    signal pic_act   : std_logic;     -- a picture is being decoded
    signal cur_fb    : unsigned(4 downto 0);
    signal pic_fn    : unsigned(15 downto 0);
    signal pic_idr   : std_logic;
    signal pic_refc  : std_logic;     -- nal_ref_idc /= 0
    signal pic_ltref : std_logic;
    signal pic_adapt : std_logic;
    signal pic_cur_long : std_logic;
    signal pic_mmco5 : std_logic;
    signal first_sl  : std_logic;     -- slice is the first of its picture
    type mmco_op_t is array (0 to 15) of unsigned(2 downto 0);
    type mmco_a_t  is array (0 to 15) of unsigned(16 downto 0);
    type mmco_b_t  is array (0 to 15) of unsigned(4 downto 0);
    signal mmco_op  : mmco_op_t;
    signal mmco_a   : mmco_a_t;
    signal mmco_b   : mmco_b_t;
    signal mmco_n   : unsigned(4 downto 0);
    signal mmco_k   : unsigned(4 downto 0);
    signal mmco_cur : unsigned(2 downto 0);
    -- sequential scans over the frame buffers (one entry per cycle)
    signal si       : natural range 0 to C_NUM_FB;
    signal best_i   : integer range -1 to C_NUM_FB - 1;
    signal best_v   : signed(17 downto 0);
    signal cnt_ref  : natural range 0 to C_NUM_FB;
    signal pnum_r   : signed(17 downto 0);

    signal consume  : unsigned(6 downto 0);
    signal cons_d1  : unsigned(6 downto 0);  -- consume of the previous cycle
    -- registered window decode (header syntax is not time critical: 3 cycles per element)
    signal w_q      : std_logic_vector(63 downto 0);
    signal eg_q     : eg_t;
    signal av_q     : std_logic;
    signal flush    : std_logic;
    signal own      : std_logic;
    signal sstart   : std_logic;
    signal stream_done : std_logic;
    signal pic_count : unsigned(31 downto 0);

    -- frame number wrap relative to the current frame_num
    function pic_num(fn, cur : unsigned(15 downto 0); log2fn : unsigned(4 downto 0)) return signed is
        variable r : signed(17 downto 0);
    begin
        if fn > cur then
            r := signed(resize(fn, 18)) - shift_left(to_signed(1, 18), to_integer(log2fn));
        else
            r := signed(resize(fn, 18));
        end if;
        return r;
    end function;

begin
    st_dbg <= state_t'pos(st);
    hdr_consume <= consume;
    br_flush <= flush;
    own_br <= own;
    slice <= sl;
    slice_start <= sstart;
    o_error <= err;
    o_stream_done <= stream_done;
    o_pic_count <= pic_count;
    o_mb_w <= act_sps.mb_w;
    o_mb_h <= act_sps.mb_h;
    o_crop_l <= act_sps.crop_l;
    o_crop_r <= act_sps.crop_r;
    o_crop_t <= act_sps.crop_t;
    o_crop_b <= act_sps.crop_b;

    -- parameter set RAMs
    process (clk)
    begin
        if rising_edge(clk) then
            if sps_we = '1' then
                sps_ram(to_integer(sps_wa)) <= sps_wd;
            end if;
            if pps_we = '1' then
                pps_ram(to_integer(pps_wa)) <= pps_wd;
            end if;
            sps_rd <= sps_ram(to_integer(sps_ra));
            pps_rd <= pps_ram(to_integer(pps_ra));
        end if;
    end process;

    -- frame output
    o_frame_valid <= '1' when oq_cnt /= 0 and dpb_wr(to_integer(oq(0))) = '1' else '0';
    o_frame_fb <= oq(0);

    process (clk)
        variable eg     : eg_t;
        variable ue     : unsigned(31 downto 0);
        variable se     : signed(31 downto 0);
        variable rdy    : boolean;
        variable cons   : unsigned(6 downto 0);
        variable w      : std_logic_vector(63 downto 0);
        variable pn_si  : signed(17 downto 0);
        variable op     : natural range 0 to 7;
        variable best   : integer;
        variable n      : integer;
        variable cntref : integer;
        variable nwrap  : signed(17 downto 0);
        variable pnum   : signed(17 downto 0);
        variable maxfn  : signed(17 downto 0);
        variable found  : boolean;
        variable oq_v   : oq_t;
        variable oqc_v  : unsigned(2 downto 0);
        variable ref_v  : ref_arr_t;
        variable lt_v   : lt_arr_t;

        function ubits(win : std_logic_vector(63 downto 0); nb : natural) return unsigned is
            variable r : unsigned(31 downto 0) := (others => '0');
        begin
            for i in 0 to 31 loop
                if i < nb then
                    r(i) := win(64 - nb + i);
                end if;
            end loop;
            return r;
        end function;
    begin
        if rising_edge(clk) then
            sps_we <= '0';
            pps_we <= '0';
            flush <= '0';
            sstart <= '0';
            cons := (others => '0');
            w_q <= br_win;
            eg_q <= ue_decode(br_win);
            av_q <= br_avail64;
            cons_d1 <= consume;
            w := w_q;
            eg := eg_q;
            ue := eg.val;
            se := se_from_ue(eg.val);
            -- consume is registered and the window decode is registered: a read
            -- is allowed when no consume was issued in the last two cycles
            rdy := av_q = '1' and own = '1' and flush = '0' and consume = 0 and cons_d1 = 0;
            maxfn := shift_left(to_signed(1, 18), to_integer(act_sps.log2fn));
            oq_v := oq;
            oqc_v := oq_cnt;

            -- written pictures
            if pic_done = '1' then
                dpb_wr(to_integer(pic_done_fb)) <= '1';
            end if;
            -- frame handed to display
            if oq_cnt /= 0 and dpb_wr(to_integer(oq(0))) = '1' and i_frame_ready = '1' then
                dpb_disp(to_integer(oq(0))) <= '1';
                dpb_outp(to_integer(oq(0))) <= '0';
                oq_v(0 to 2) := oq(1 to 3);
                oqc_v := oq_cnt - 1;
            end if;
            if i_release = '1' then
                dpb_disp(to_integer(i_release_fb)) <= '0';
            end if;

            if rst = '1' then
                st <= S_IDLE;
                own <= '1';
                err <= '0';
                stream_done <= '0';
                pic_act <= '0';
                cur_fb <= (others => '0');
                dpb_ref <= (others => REF_NONE);
                dpb_fn <= (others => (others => '0'));
                dpb_lt <= (others => (others => '0'));
                dpb_disp <= (others => '0');
                dpb_outp <= (others => '0');
                dpb_wr <= (others => '0');
                max_lt <= to_signed(-1, 6);
                oq_cnt <= (others => '0');
                oq <= (others => (others => '0'));
                act_sps <= SPS_INIT;
                act_pps <= PPS_INIT;
                rl <= (others => (others => '0'));
                rl_len <= (others => '0');
                sl <= SLICE_INIT;
                pic_count <= (others => '0');
                mmco_n <= (others => '0');
            else
                case st is
                    -- ------------------------------------------------
                    when S_IDLE =>
                        own <= '1';
                        if br_active = '1' and br_avail64 = '1' then
                            st <= S_NALHDR;
                        elsif br_active = '0' and br_eos = '1' then
                            st <= S_EOS_FINISH;
                        end if;

                    when S_NALHDR =>
                        if rdy then
                            cons := to_unsigned(8, 7);
                            nal_ref <= unsigned(w(62 downto 61));
                            nal_type <= unsigned(w(60 downto 56));
                            case to_integer(unsigned(w(60 downto 56))) is
                                when 7 =>
                                    st <= S_SPS_PROF;
                                    t_sps <= SPS_INIT;
                                when 8 =>
                                    st <= S_PPS_ID;
                                    t_pps <= PPS_INIT;
                                when 1 | 5 =>
                                    st <= S_SL_FIRST;
                                    sl_idr <= to_sl(unsigned(w(60 downto 56)) = 5);
                                when others =>
                                    st <= S_END_NAL;
                            end case;
                        end if;

                    when S_END_NAL =>
                        -- drop the rest of the NAL
                        flush <= '1';
                        st <= S_WAIT_FLUSH;

                    when S_WAIT_FLUSH =>
                        st <= S_IDLE;

                    -- ------------------------------------------------
                    -- sequence parameter set
                    -- ------------------------------------------------
                    when S_SPS_PROF =>
                        if rdy then
                            cons := to_unsigned(8, 7);
                            case to_integer(unsigned(w(63 downto 56))) is
                                when 100 | 110 | 122 | 244 | 44 | 83 | 86 | 118 | 128 | 138 | 139 | 134 | 135 =>
                                    hi_prof <= '1';
                                when others =>
                                    hi_prof <= '0';
                            end case;
                            st <= S_SPS_CONS;
                        end if;
                    when S_SPS_CONS =>
                        if rdy then
                            cons := to_unsigned(8, 7);
                            st <= S_SPS_LEVEL;
                        end if;
                    when S_SPS_LEVEL =>
                        if rdy then
                            cons := to_unsigned(8, 7);
                            st <= S_SPS_ID;
                        end if;
                    when S_SPS_ID =>
                        if rdy then
                            cons := eg.len;
                            t_id <= resize(ue(4 downto 0), 8);
                            if ue > 31 then
                                st <= S_END_NAL;
                                err <= '1';
                            elsif hi_prof = '1' then
                                st <= S_SPS_CF;
                            else
                                st <= S_SPS_LOG2FN;
                            end if;
                        end if;
                    when S_SPS_CF =>
                        if rdy then
                            cons := eg.len;
                            if ue /= 1 then
                                t_sps.unsup <= '1';
                            end if;
                            if ue = 3 then
                                -- separate_colour_plane_flag: not supported, stop here
                                st <= S_SPS_STORE;
                            else
                                st <= S_SPS_BDY;
                            end if;
                        end if;
                    when S_SPS_BDY =>
                        if rdy then
                            cons := eg.len;
                            if ue /= 0 then
                                t_sps.unsup <= '1';
                            end if;
                            st <= S_SPS_BDC;
                        end if;
                    when S_SPS_BDC =>
                        if rdy then
                            cons := eg.len;
                            if ue /= 0 then
                                t_sps.unsup <= '1';
                            end if;
                            st <= S_SPS_BYP;
                        end if;
                    when S_SPS_BYP =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                t_sps.unsup <= '1';      -- lossless
                            end if;
                            st <= S_SPS_SCAL;
                        end if;
                    when S_SPS_SCAL =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                t_sps.unsup <= '1';      -- scaling matrices
                                st <= S_SPS_STORE;
                            else
                                st <= S_SPS_LOG2FN;
                            end if;
                        end if;
                    when S_SPS_LOG2FN =>
                        if rdy then
                            cons := eg.len;
                            t_sps.log2fn <= resize(ue(3 downto 0), 5) + 4;
                            if ue > 12 then
                                t_sps.unsup <= '1';
                            end if;
                            st <= S_SPS_POCT;
                        end if;
                    when S_SPS_POCT =>
                        if rdy then
                            cons := eg.len;
                            t_sps.poc_type <= ue(1 downto 0);
                            if ue = 0 then
                                st <= S_SPS_LOG2POC;
                            elsif ue = 1 then
                                st <= S_SPS_DPOAZ;
                            else
                                st <= S_SPS_NRF;
                            end if;
                        end if;
                    when S_SPS_LOG2POC =>
                        if rdy then
                            cons := eg.len;
                            t_sps.log2poc <= resize(ue(3 downto 0), 5) + 4;
                            st <= S_SPS_NRF;
                        end if;
                    when S_SPS_DPOAZ =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            t_sps.dpoaz <= w(63);
                            st <= S_SPS_ONRP;
                        end if;
                    when S_SPS_ONRP =>
                        if rdy then
                            cons := eg.len;
                            st <= S_SPS_OTTB;
                        end if;
                    when S_SPS_OTTB =>
                        if rdy then
                            cons := eg.len;
                            st <= S_SPS_NCYC;
                        end if;
                    when S_SPS_NCYC =>
                        if rdy then
                            cons := eg.len;
                            cnt <= resize(ue(8 downto 0), 9);
                            if ue = 0 then
                                st <= S_SPS_NRF;
                            else
                                st <= S_SPS_OFS;
                            end if;
                        end if;
                    when S_SPS_OFS =>
                        if rdy then
                            cons := eg.len;
                            cnt <= cnt - 1;
                            if cnt = 1 then
                                st <= S_SPS_NRF;
                            end if;
                        end if;
                    when S_SPS_NRF =>
                        if rdy then
                            cons := eg.len;
                            t_sps.nrf <= ue(4 downto 0);
                            if ue > 16 then
                                t_sps.unsup <= '1';
                            end if;
                            st <= S_SPS_GAPS;
                        end if;
                    when S_SPS_GAPS =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            st <= S_SPS_W;
                        end if;
                    when S_SPS_W =>
                        if rdy then
                            cons := eg.len;
                            t_sps.mb_w <= ue(7 downto 0) + 1;
                            if ue >= C_MAX_MB_W then
                                t_sps.unsup <= '1';
                            end if;
                            st <= S_SPS_H;
                        end if;
                    when S_SPS_H =>
                        if rdy then
                            cons := eg.len;
                            t_sps.mb_h <= ue(7 downto 0) + 1;
                            if ue >= C_MAX_MB_H then
                                t_sps.unsup <= '1';
                            end if;
                            st <= S_SPS_FMO;
                        end if;
                    when S_SPS_FMO =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '0' then
                                t_sps.unsup <= '1';      -- interlace
                                st <= S_SPS_STORE;
                            else
                                st <= S_SPS_D8;
                            end if;
                        end if;
                    when S_SPS_D8 =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            st <= S_SPS_CROP;
                        end if;
                    when S_SPS_CROP =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                st <= S_SPS_CL;
                            else
                                st <= S_SPS_STORE;
                            end if;
                        end if;
                    when S_SPS_CL =>
                        if rdy then
                            cons := eg.len;
                            t_sps.crop_l <= ue(9 downto 0);
                            st <= S_SPS_CR;
                        end if;
                    when S_SPS_CR =>
                        if rdy then
                            cons := eg.len;
                            t_sps.crop_r <= ue(9 downto 0);
                            st <= S_SPS_CT;
                        end if;
                    when S_SPS_CT =>
                        if rdy then
                            cons := eg.len;
                            t_sps.crop_t <= ue(9 downto 0);
                            st <= S_SPS_CB;
                        end if;
                    when S_SPS_CB =>
                        if rdy then
                            cons := eg.len;
                            t_sps.crop_b <= ue(9 downto 0);
                            st <= S_SPS_STORE;
                        end if;
                    when S_SPS_STORE =>
                        sps_we <= '1';
                        sps_wa <= t_id(4 downto 0);
                        sps_wd <= pack_sps(t_sps);
                        sps_wd(SPS_BITS - 1) <= '1';
                        if t_sps.unsup = '1' then
                            err <= '1';
                        end if;
                        st <= S_END_NAL;

                    -- ------------------------------------------------
                    -- picture parameter set
                    -- ------------------------------------------------
                    when S_PPS_ID =>
                        if rdy then
                            cons := eg.len;
                            t_id <= ue(7 downto 0);
                            if ue > 255 then
                                st <= S_END_NAL;
                                err <= '1';
                            else
                                st <= S_PPS_SPSID;
                            end if;
                        end if;
                    when S_PPS_SPSID =>
                        if rdy then
                            cons := eg.len;
                            t_pps.sps_id <= ue(4 downto 0);
                            if ue > 31 then
                                t_pps.unsup <= '1';
                            end if;
                            st <= S_PPS_ENT;
                        end if;
                    when S_PPS_ENT =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                t_pps.unsup <= '1';      -- CABAC
                            end if;
                            st <= S_PPS_BFPO;
                        end if;
                    when S_PPS_BFPO =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            t_pps.bfpo <= w(63);
                            st <= S_PPS_NSG;
                        end if;
                    when S_PPS_NSG =>
                        if rdy then
                            cons := eg.len;
                            if ue /= 0 then
                                t_pps.unsup <= '1';      -- FMO
                                st <= S_PPS_STORE;
                            else
                                st <= S_PPS_NREF0;
                            end if;
                        end if;
                    when S_PPS_NREF0 =>
                        if rdy then
                            cons := eg.len;
                            t_pps.nref <= ue(5 downto 0) + 1;
                            if ue > 31 then
                                t_pps.unsup <= '1';
                            end if;
                            st <= S_PPS_NREF1;
                        end if;
                    when S_PPS_NREF1 =>
                        if rdy then
                            cons := eg.len;
                            st <= S_PPS_WP;
                        end if;
                    when S_PPS_WP =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                t_pps.unsup <= '1';      -- weighted prediction
                            end if;
                            st <= S_PPS_WB;
                        end if;
                    when S_PPS_WB =>
                        if rdy then
                            cons := to_unsigned(2, 7);
                            st <= S_PPS_QP;
                        end if;
                    when S_PPS_QP =>
                        if rdy then
                            cons := eg.len;
                            t_pps.init_qp <= to_unsigned(clip3(0, 51, to_integer(se) + 26), 6);
                            if se < -26 or se > 25 then
                                t_pps.unsup <= '1';
                            end if;
                            st <= S_PPS_QS;
                        end if;
                    when S_PPS_QS =>
                        if rdy then
                            cons := eg.len;
                            st <= S_PPS_CQP;
                        end if;
                    when S_PPS_CQP =>
                        if rdy then
                            cons := eg.len;
                            t_pps.cqp_off <= resize(se, 5);
                            if se < -12 or se > 12 then
                                t_pps.unsup <= '1';
                            end if;
                            st <= S_PPS_DBC;
                        end if;
                    when S_PPS_DBC =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            t_pps.dbctl <= w(63);
                            st <= S_PPS_CIP;
                        end if;
                    when S_PPS_CIP =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            t_pps.cip <= w(63);
                            st <= S_PPS_RPC;
                        end if;
                    when S_PPS_RPC =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            t_pps.rpc <= w(63);
                            st <= S_PPS_T8;
                        end if;
                    when S_PPS_T8 =>
                        -- optional High profile extension
                        if rdy then
                            if br_more = '1' then
                                cons := to_unsigned(1, 7);
                                if w(63) = '1' then
                                    t_pps.unsup <= '1';  -- 8x8 transform
                                end if;
                                st <= S_PPS_PSM;
                            else
                                st <= S_PPS_STORE;
                            end if;
                        end if;
                    when S_PPS_PSM =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                t_pps.unsup <= '1';      -- scaling matrices
                                st <= S_PPS_STORE;
                            else
                                st <= S_PPS_STORE;       -- second_chroma_qp_index_offset ignored
                            end if;
                        end if;
                    when S_PPS_STORE =>
                        pps_we <= '1';
                        pps_wa <= t_id;
                        pps_wd <= pack_pps(t_pps);
                        pps_wd(PPS_BITS - 1) <= '1';
                        if t_pps.unsup = '1' then
                            err <= '1';
                        end if;
                        st <= S_END_NAL;

                    -- ------------------------------------------------
                    -- slice header
                    -- ------------------------------------------------
                    when S_SL_FIRST =>
                        if rdy then
                            cons := eg.len;
                            sl_first <= ue(13 downto 0);
                            sl_skip <= '0';
                            st <= S_SL_TYPE;
                        end if;
                    when S_SL_TYPE =>
                        if rdy then
                            cons := eg.len;
                            case to_integer(ue(3 downto 0)) is
                                when 0 | 5 =>
                                    sl_type_p <= '1';
                                when 2 | 7 =>
                                    sl_type_p <= '0';
                                when others =>
                                    sl_type_p <= '0';
                                    sl_skip <= '1';
                                    err <= '1';
                            end case;
                            if ue > 9 then
                                sl_skip <= '1';
                            end if;
                            st <= S_SL_PPS;
                        end if;
                    when S_SL_PPS =>
                        if rdy then
                            cons := eg.len;
                            pps_ra <= ue(7 downto 0);
                            st <= S_SL_RDPPS;
                        end if;
                    when S_SL_RDPPS =>
                        -- RAM read latency
                        st <= S_SL_RDSPS;
                    when S_SL_RDSPS =>
                        act_pps <= unpack_pps(pps_rd);
                        sps_ra <= unpack_pps(pps_rd).sps_id;
                        st <= S_SL_RDSPS2;
                    when S_SL_RDSPS2 =>
                        -- RAM read latency (address register + output register)
                        st <= S_SL_CHECK;
                    when S_SL_CHECK =>
                        st <= S_SL_FN;
                        act_sps <= unpack_sps(sps_rd);
                        if act_pps.valid = '0' or act_pps.unsup = '1' or unpack_sps(sps_rd).valid = '0' or
                           unpack_sps(sps_rd).unsup = '1' then
                            sl_skip <= '1';
                            err <= '1';
                            st <= S_END_NAL;
                        end if;
                    when S_SL_FN =>
                        if rdy then
                            cons := resize(act_sps.log2fn, 7);
                            sl_fn <= resize(ubits(w, to_integer(act_sps.log2fn)), 16);
                            if sl_first = 0 then
                                first_sl <= '1';
                                ret_st <= S_PIC_ALLOC;
                                st <= S_PIC_FINISH;
                            else
                                first_sl <= '0';
                                if pic_act = '0' then
                                    -- slice of a picture whose start was lost
                                    sl_skip <= '1';
                                    st <= S_END_NAL;
                                elsif sl_idr = '1' then
                                    st <= S_SL_IDR;
                                else
                                    st <= S_SL_POCLSB;
                                end if;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- finish previous picture: reference marking
                    -- ------------------------------------------------
                    when S_PIC_FINISH =>
                        mmco_k <= (others => '0');
                        pic_cur_long <= '0';
                        pic_mmco5 <= '0';
                        if pic_act = '0' or pic_refc = '0' then
                            st <= ret_st;
                            pic_act <= '0';
                        elsif pic_idr = '1' then
                            for i in 0 to C_NUM_FB - 1 loop
                                dpb_ref(i) <= REF_NONE;
                            end loop;
                            if pic_ltref = '1' then
                                dpb_ref(to_integer(cur_fb)) <= REF_LONG;
                                dpb_lt(to_integer(cur_fb)) <= (others => '0');
                                max_lt <= to_signed(0, 6);
                            else
                                dpb_ref(to_integer(cur_fb)) <= REF_SHORT;
                                max_lt <= to_signed(-1, 6);
                            end if;
                            dpb_fn(to_integer(cur_fb)) <= pic_fn;
                            pic_act <= '0';
                            st <= ret_st;
                        elsif pic_adapt = '1' then
                            st <= S_MK_MMCO;
                        else
                            si <= 0;
                            cnt_ref <= 0;
                            best_i <= -1;
                            st <= S_MK_SLIDE;
                        end if;

                    when S_MK_MMCO =>
                        -- one operation per pass; the frame buffers are scanned one per cycle
                        if mmco_k = mmco_n then
                            st <= S_MK_CUR;
                        else
                            op := to_integer(mmco_op(to_integer(mmco_k)));
                            pnum_r <= signed(resize(pic_fn, 18)) - signed(resize(mmco_a(to_integer(mmco_k)), 18));
                            si <= 0;
                            if op = 4 then
                                max_lt <= signed(resize(mmco_a(to_integer(mmco_k)), 6)) - 1;
                            end if;
                            if op = 5 then
                                for i in 0 to C_NUM_FB - 1 loop
                                    if i /= to_integer(cur_fb) then
                                        dpb_ref(i) <= REF_NONE;
                                    end if;
                                end loop;
                                max_lt <= to_signed(-1, 6);
                                pic_mmco5 <= '1';
                                mmco_k <= mmco_k + 1;
                            else
                                st <= S_MK_OPSCAN;
                            end if;
                        end if;

                    when S_MK_OPSCAN =>
                        op := to_integer(mmco_op(to_integer(mmco_k)));
                        pn_si := pic_num(dpb_fn(si), pic_fn, act_sps.log2fn);
                        if si /= to_integer(cur_fb) then
                            case op is
                                when 1 =>
                                    if dpb_ref(si) = REF_SHORT and pn_si = pnum_r then
                                        dpb_ref(si) <= REF_NONE;
                                    end if;
                                when 2 =>
                                    if dpb_ref(si) = REF_LONG and resize(dpb_lt(si), 17) = mmco_a(to_integer(mmco_k)) then
                                        dpb_ref(si) <= REF_NONE;
                                    end if;
                                when 3 =>
                                    if dpb_ref(si) = REF_LONG and dpb_lt(si) = mmco_b(to_integer(mmco_k)) then
                                        dpb_ref(si) <= REF_NONE;
                                    end if;
                                    if dpb_ref(si) = REF_SHORT and pn_si = pnum_r then
                                        dpb_ref(si) <= REF_LONG;
                                        dpb_lt(si) <= mmco_b(to_integer(mmco_k));
                                    end if;
                                when 4 =>
                                    if dpb_ref(si) = REF_LONG and resize(dpb_lt(si), 17) >= mmco_a(to_integer(mmco_k)) then
                                        dpb_ref(si) <= REF_NONE;
                                    end if;
                                when 6 =>
                                    if dpb_ref(si) = REF_LONG and dpb_lt(si) = mmco_b(to_integer(mmco_k)) then
                                        dpb_ref(si) <= REF_NONE;
                                    end if;
                                when others =>
                                    null;
                            end case;
                        end if;
                        if si = C_NUM_FB - 1 then
                            if op = 6 then
                                dpb_ref(to_integer(cur_fb)) <= REF_LONG;
                                dpb_lt(to_integer(cur_fb)) <= mmco_b(to_integer(mmco_k));
                                pic_cur_long <= '1';
                            end if;
                            mmco_k <= mmco_k + 1;
                            st <= S_MK_MMCO;
                        else
                            si <= si + 1;
                        end if;

                    when S_MK_SLIDE =>
                        -- count references, find the short-term one with the smallest FrameNumWrap
                        pn_si := pic_num(dpb_fn(si), pic_fn, act_sps.log2fn);
                        cntref := cnt_ref;
                        best := best_i;
                        if si /= to_integer(cur_fb) and dpb_ref(si) /= REF_NONE then
                            cntref := cntref + 1;
                        end if;
                        if si /= to_integer(cur_fb) and dpb_ref(si) = REF_SHORT and (best < 0 or pn_si < best_v) then
                            best := si;
                            best_v <= pn_si;
                        end if;
                        cnt_ref <= cntref;
                        best_i <= best;
                        if si = C_NUM_FB - 1 then
                            if cntref >= maximum(to_integer(act_sps.nrf), 1) and best >= 0 then
                                dpb_ref(best) <= REF_NONE;
                            end if;
                            st <= S_MK_CUR;
                        else
                            si <= si + 1;
                        end if;

                    when S_MK_CUR =>
                        if pic_cur_long = '0' then
                            dpb_ref(to_integer(cur_fb)) <= REF_SHORT;
                        end if;
                        if pic_mmco5 = '1' then
                            dpb_fn(to_integer(cur_fb)) <= (others => '0');
                        else
                            dpb_fn(to_integer(cur_fb)) <= pic_fn;
                        end if;
                        pic_act <= '0';
                        st <= ret_st;

                    -- ------------------------------------------------
                    -- start a new picture: allocate a frame buffer
                    -- ------------------------------------------------
                    when S_PIC_ALLOC =>
                        best := -1;
                        for i in C_NUM_FB - 1 downto 0 loop
                            if dpb_ref(i) = REF_NONE and dpb_disp(i) = '0' and dpb_outp(i) = '0' then
                                best := i;
                            end if;
                        end loop;
                        if best >= 0 and oqc_v < 4 then
                            cur_fb <= to_unsigned(best, 5);
                            dpb_outp(best) <= '1';
                            dpb_wr(best) <= '0';
                            dpb_ref(best) <= REF_NONE;
                            oq_v(to_integer(oqc_v)) := to_unsigned(best, 5);
                            oqc_v := oqc_v + 1;
                            pic_act <= '1';
                            pic_fn <= sl_fn;
                            pic_idr <= sl_idr;
                            pic_refc <= to_sl(nal_ref /= 0);
                            pic_ltref <= '0';
                            pic_adapt <= '0';
                            mmco_n <= (others => '0');
                            pic_count <= pic_count + 1;
                            if sl_idr = '1' then
                                st <= S_SL_IDR;
                            else
                                st <= S_SL_POCLSB;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    when S_SL_IDR =>
                        if rdy then
                            cons := eg.len;
                            st <= S_SL_POCLSB;
                        end if;
                    when S_SL_POCLSB =>
                        if act_sps.poc_type = 0 then
                            if rdy then
                                cons := resize(act_sps.log2poc, 7);
                                if act_pps.bfpo = '1' then
                                    st <= S_SL_DPOCB;
                                else
                                    st <= S_SL_RPC;
                                end if;
                            end if;
                        elsif act_sps.poc_type = 1 and act_sps.dpoaz = '0' then
                            st <= S_SL_DPOC0;
                        else
                            st <= S_SL_RPC;
                        end if;
                    when S_SL_DPOCB =>
                        if rdy then
                            cons := eg.len;
                            st <= S_SL_RPC;
                        end if;
                    when S_SL_DPOC0 =>
                        if rdy then
                            cons := eg.len;
                            if act_pps.bfpo = '1' then
                                st <= S_SL_DPOC1;
                            else
                                st <= S_SL_RPC;
                            end if;
                        end if;
                    when S_SL_DPOC1 =>
                        if rdy then
                            cons := eg.len;
                            st <= S_SL_RPC;
                        end if;
                    when S_SL_RPC =>
                        if act_pps.rpc = '1' then
                            if rdy then
                                cons := eg.len;
                                if ue /= 0 then
                                    sl_skip <= '1';      -- redundant picture
                                    st <= S_END_NAL;
                                else
                                    st <= S_SL_NROVR;
                                end if;
                            end if;
                        else
                            st <= S_SL_NROVR;
                        end if;
                        sl_nref <= act_pps.nref;
                    when S_SL_NROVR =>
                        if sl_type_p = '0' then
                            st <= S_SL_NOOUT;
                        elsif rdy then
                            cons := to_unsigned(1, 7);
                            if w(63) = '1' then
                                st <= S_SL_NREF;
                            else
                                st <= S_RL_INIT;
                            end if;
                        end if;
                    when S_SL_NREF =>
                        if rdy then
                            cons := eg.len;
                            sl_nref <= ue(5 downto 0) + 1;
                            if ue > 15 then
                                sl_skip <= '1';
                                err <= '1';
                                st <= S_END_NAL;
                            else
                                st <= S_RL_INIT;
                            end if;
                        end if;

                    -- ------------------------------------------------
                    -- reference list initialisation (P slice)
                    -- ------------------------------------------------
                    when S_RL_INIT =>
                        rl_taken <= (others => '0');
                        rl_len <= (others => '0');
                        for i in 0 to 16 loop
                            rl(i) <= FB_NONE;
                        end loop;
                        if sl_nref > 16 then
                            sl_nref <= to_unsigned(16, 6);
                        end if;
                        si <= 0;
                        best_i <= -1;
                        st <= S_RL_SHORT;
                    when S_RL_SHORT =>
                        -- pick the not yet listed short-term picture with the largest PicNum
                        pn_si := pic_num(dpb_fn(si), sl_fn, act_sps.log2fn);
                        best := best_i;
                        if si /= to_integer(cur_fb) and dpb_ref(si) = REF_SHORT and rl_taken(si) = '0' and
                           (best < 0 or pn_si > best_v) then
                            best := si;
                            best_v <= pn_si;
                        end if;
                        best_i <= best;
                        if si = C_NUM_FB - 1 then
                            si <= 0;
                            best_i <= -1;
                            if best >= 0 and rl_len < 16 then
                                rl(to_integer(rl_len)) <= to_unsigned(best, 5);
                                rl_taken(best) <= '1';
                                rl_len <= rl_len + 1;
                            else
                                st <= S_RL_LONG;
                            end if;
                        else
                            si <= si + 1;
                        end if;
                    when S_RL_LONG =>
                        -- then long-term pictures by ascending LongTermPicNum
                        best := best_i;
                        if si /= to_integer(cur_fb) and dpb_ref(si) = REF_LONG and rl_taken(si) = '0' and
                           (best < 0 or signed(resize(dpb_lt(si), 18)) < best_v) then
                            best := si;
                            best_v <= signed(resize(dpb_lt(si), 18));
                        end if;
                        best_i <= best;
                        if si = C_NUM_FB - 1 then
                            si <= 0;
                            best_i <= -1;
                            if best >= 0 and rl_len < 16 then
                                rl(to_integer(rl_len)) <= to_unsigned(best, 5);
                                rl_taken(best) <= '1';
                                rl_len <= rl_len + 1;
                            else
                                st <= S_SL_RPLM;
                            end if;
                        else
                            si <= si + 1;
                        end if;
                    when S_SL_RPLM =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            rl_idx <= (others => '0');
                            rl_pred <= signed(resize(sl_fn, 18));
                            if w(63) = '1' then
                                st <= S_SL_MODIDC;
                            else
                                st <= S_SL_NOOUT;
                            end if;
                        end if;
                    when S_SL_MODIDC =>
                        if rdy then
                            cons := eg.len;
                            mod_idc <= ue(1 downto 0);
                            if ue = 3 then
                                st <= S_SL_NOOUT;
                            elsif ue > 3 or rl_idx >= 16 then
                                err <= '1';
                                sl_skip <= '1';
                                st <= S_END_NAL;
                            else
                                st <= S_SL_MODVAL;
                            end if;
                        end if;
                    when S_SL_MODVAL =>
                        if rdy then
                            cons := eg.len;
                            if mod_idc = 2 then
                                pnum := signed(resize(ue(16 downto 0), 18));
                            else
                                if mod_idc = 0 then
                                    nwrap := rl_pred - signed(resize(ue(16 downto 0), 18)) - 1;
                                    if nwrap < 0 then
                                        nwrap := nwrap + maxfn;
                                    end if;
                                else
                                    nwrap := rl_pred + signed(resize(ue(16 downto 0), 18)) + 1;
                                    if nwrap >= maxfn then
                                        nwrap := nwrap - maxfn;
                                    end if;
                                end if;
                                rl_pred <= nwrap;
                                if nwrap > signed(resize(sl_fn, 18)) then
                                    pnum := nwrap - maxfn;
                                else
                                    pnum := nwrap;
                                end if;
                            end if;
                            pnum_r <= pnum;
                            si <= 0;
                            best_i <= -1;
                            st <= S_RL_MODFIND;
                        end if;
                    when S_RL_MODFIND =>
                        -- find the picture with PicNum / LongTermPicNum = pnum_r
                        pn_si := pic_num(dpb_fn(si), sl_fn, act_sps.log2fn);
                        best := best_i;
                        if si /= to_integer(cur_fb) then
                            if mod_idc = 2 then
                                if dpb_ref(si) = REF_LONG and signed(resize(dpb_lt(si), 18)) = pnum_r then
                                    best := si;
                                end if;
                            elsif dpb_ref(si) = REF_SHORT and pn_si = pnum_r then
                                best := si;
                            end if;
                        end if;
                        best_i <= best;
                        if si = C_NUM_FB - 1 then
                            if best < 0 then
                                err <= '1';
                                rl_tgt <= FB_NONE;
                            else
                                rl_tgt <= to_unsigned(best, 5);
                            end if;
                            st <= S_RL_MODSHIFT;
                        else
                            si <= si + 1;
                        end if;
                    when S_RL_MODSHIFT =>
                        -- RefPicList[cIdx] = RefPicList[cIdx-1] for cIdx > refIdx; insert
                        for c in 1 to 16 loop
                            if c > to_integer(rl_idx) and c <= to_integer(sl_nref) then
                                rl(c) <= rl(c - 1);
                            end if;
                        end loop;
                        rl(to_integer(rl_idx)) <= rl_tgt;
                        rl_idx <= rl_idx + 1;
                        rl_c <= rl_idx + 1;
                        rl_n <= rl_idx + 1;
                        st <= S_RL_COMPACT;
                    when S_RL_COMPACT =>
                        -- remove the duplicate of the inserted picture behind refIdx
                        if rl_c > resize(sl_nref, 5) then
                            st <= S_SL_MODIDC;
                        else
                            if rl(to_integer(rl_c)) /= rl_tgt or rl_tgt = FB_NONE then
                                rl(to_integer(rl_n)) <= rl(to_integer(rl_c));
                                rl_n <= rl_n + 1;
                            end if;
                            rl_c <= rl_c + 1;
                        end if;

                    -- ------------------------------------------------
                    -- dec_ref_pic_marking
                    -- ------------------------------------------------
                    when S_SL_NOOUT =>
                        if nal_ref = 0 then
                            st <= S_SL_QPD;
                        elsif sl_idr = '1' then
                            if rdy then
                                cons := to_unsigned(1, 7);
                                st <= S_SL_LTREF;
                            end if;
                        else
                            st <= S_SL_ADAPT;
                        end if;
                    when S_SL_LTREF =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if first_sl = '1' then
                                pic_ltref <= w(63);
                            end if;
                            st <= S_SL_QPD;
                        end if;
                    when S_SL_ADAPT =>
                        if rdy then
                            cons := to_unsigned(1, 7);
                            if first_sl = '1' then
                                pic_adapt <= w(63);
                                mmco_n <= (others => '0');
                            end if;
                            if w(63) = '1' then
                                st <= S_SL_MMCO;
                            else
                                st <= S_SL_QPD;
                            end if;
                        end if;
                    when S_SL_MMCO =>
                        if rdy then
                            cons := eg.len;
                            mmco_cur <= ue(2 downto 0);
                            if ue = 0 then
                                st <= S_SL_QPD;
                            elsif ue > 6 then
                                err <= '1';
                                sl_skip <= '1';
                                st <= S_END_NAL;
                            else
                                if first_sl = '1' then
                                    mmco_op(to_integer(mmco_n(3 downto 0))) <= ue(2 downto 0);
                                    mmco_a(to_integer(mmco_n(3 downto 0))) <= (others => '0');
                                    mmco_b(to_integer(mmco_n(3 downto 0))) <= (others => '0');
                                end if;
                                if ue = 5 then
                                    if first_sl = '1' then
                                        mmco_n <= mmco_n + 1;
                                    end if;
                                elsif ue = 6 then
                                    st <= S_SL_MMCO_B;
                                else
                                    st <= S_SL_MMCO_A;
                                end if;
                            end if;
                        end if;
                    when S_SL_MMCO_A =>
                        if rdy then
                            cons := eg.len;
                            if first_sl = '1' then
                                if mmco_cur = 1 or mmco_cur = 3 then
                                    mmco_a(to_integer(mmco_n(3 downto 0))) <= ue(16 downto 0) + 1;
                                else
                                    mmco_a(to_integer(mmco_n(3 downto 0))) <= ue(16 downto 0);
                                end if;
                            end if;
                            if mmco_cur = 3 then
                                st <= S_SL_MMCO_B;
                            else
                                if first_sl = '1' then
                                    mmco_n <= mmco_n + 1;
                                end if;
                                st <= S_SL_MMCO;
                            end if;
                        end if;
                    when S_SL_MMCO_B =>
                        if rdy then
                            cons := eg.len;
                            if first_sl = '1' then
                                mmco_b(to_integer(mmco_n(3 downto 0))) <= ue(4 downto 0);
                                mmco_n <= mmco_n + 1;
                            end if;
                            if mmco_n = 15 then
                                err <= '1';      -- more MMCO operations than stored
                            end if;
                            st <= S_SL_MMCO;
                        end if;

                    -- ------------------------------------------------
                    when S_SL_QPD =>
                        if rdy then
                            cons := eg.len;
                            sl.qp <= to_unsigned(clip3(0, 51, to_integer(act_pps.init_qp) + to_integer(se)), 6);
                            sl.disable_db <= "00";
                            sl.alpha_off <= (others => '0');
                            sl.beta_off <= (others => '0');
                            if act_pps.dbctl = '1' then
                                st <= S_SL_DDF;
                            else
                                st <= S_SL_START;
                            end if;
                        end if;
                    when S_SL_DDF =>
                        if rdy then
                            cons := eg.len;
                            sl.disable_db <= ue(1 downto 0);
                            if ue = 1 then
                                st <= S_SL_START;
                            else
                                st <= S_SL_ALPHA;
                            end if;
                        end if;
                    when S_SL_ALPHA =>
                        if rdy then
                            cons := eg.len;
                            sl.alpha_off <= resize(se, 4) & '0';
                            st <= S_SL_BETA;
                        end if;
                    when S_SL_BETA =>
                        if rdy then
                            cons := eg.len;
                            sl.beta_off <= resize(se, 4) & '0';
                            st <= S_SL_START;
                        end if;

                    when S_SL_START =>
                        sl.first_mb <= sl_first;
                        sl.is_p <= sl_type_p;
                        sl.num_ref <= resize(sl_nref, 5);
                        for i in 0 to 15 loop
                            sl.ref_list(i) <= rl(i);
                        end loop;
                        sl.chroma_qp_off <= act_pps.cqp_off;
                        sl.constr_intra <= act_pps.cip;
                        sl.mb_w <= act_sps.mb_w;
                        sl.mb_h <= act_sps.mb_h;
                        sl.cur_fb <= cur_fb;
                        if sl_skip = '1' then
                            st <= S_END_NAL;
                        else
                            sstart <= '1';
                            own <= '0';
                            st <= S_SL_WAIT;
                        end if;
                    when S_SL_WAIT =>
                        if slice_done = '1' then
                            own <= '1';
                            st <= S_END_NAL;
                        end if;

                    -- ------------------------------------------------
                    when S_EOS_FINISH =>
                        ret_st <= S_DONE;
                        st <= S_PIC_FINISH;
                    when S_DONE =>
                        if oq_cnt = 0 then
                            stream_done <= '1';
                        end if;
                end case;
            end if;

            if rst = '0' then
                oq <= oq_v;
                oq_cnt <= oqc_v;
            end if;
            consume <= cons;
            if rst = '1' then
                consume <= (others => '0');
            end if;
        end if;
    end process;

end architecture;
