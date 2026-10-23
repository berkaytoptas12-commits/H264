-- ============================================================================
-- h264_disp.vhd
-- Display path: decoded frames in the DDR frame store -> HDMI sink with DVI
-- signalling (TMDS, RGB 8 bit), four pixels per clock for the PolarFire
-- transceiver in PMA mode (40 bit lanes, lane rate = 10 x pixel clock).
--
--   h264_disp_rd  (decoder clock): takes the decoded frames, picks one per
--                  display frame (frame rate pacing), reads its lines from
--                  DDR (AXI4, own read port) into 4 line slots
--   h264_disp_px  (pixel clock = transceiver TX clock / 1, 4 pixels per
--                  clock): video timing, YCbCr 4:2:0 -> RGB, colour bars,
--                  TMDS encoding of the three channels and the clock lane
--   h264_disp     wrapper: line slot RAMs, clock domain crossings
--
-- The picture is centred (or centre-cropped) on the display, no scaling.
-- Without a decoded frame, while switched off or with i_ctrl(1) the
-- display shows 75 % colour bars.  Frames are paced by the stream frame
-- rate (i_ctrl(31:16) = fps * 100, refresh assumed 60 Hz); a late decoder
-- repeats frames, a fast one is held back through o_frame_ready.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package h264_disp_pkg is
    constant DISP_SLOTS : natural := 4;
    -- slot meta data: 25 pattern, 24 blank, 23 BT.709, 22..12 x_start, 11..0 x_len
    constant META_W     : natural := 26;
    subtype meta_t is std_logic_vector(META_W - 1 downto 0);
    type meta_arr_t is array (0 to DISP_SLOTS - 1) of meta_t;
end package;

-- ============================================================================
-- reader (decoder clock)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_disp_pkg.all;

entity h264_disp_rd is
    generic (
        G_DISP_W : positive := 1920;
        G_DISP_H : positive := 1080
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;
        i_lines_en      : in  std_logic;    -- pixel clock running: fetch lines
        i_flush         : in  std_logic;    -- decoder reset: forget all frames (no release)
        i_show          : in  std_logic;    -- 0: frames are released at once
        i_pattern       : in  std_logic;    -- colour bars instead of the picture
        i_rate          : in  std_logic_vector(15 downto 0);   -- fps * 100, 0 = 30
        -- decoded frames
        i_frame_valid   : in  std_logic;
        i_frame_index   : in  std_logic_vector(4 downto 0);
        i_frame_addr_y  : in  std_logic_vector(31 downto 0);
        i_frame_addr_cb : in  std_logic_vector(31 downto 0);
        i_frame_addr_cr : in  std_logic_vector(31 downto 0);
        o_frame_ready   : out std_logic;
        o_release       : out std_logic;
        o_release_index : out std_logic_vector(4 downto 0);
        i_width         : in  std_logic_vector(11 downto 0);
        i_height        : in  std_logic_vector(11 downto 0);
        i_crop_left     : in  std_logic_vector(10 downto 0);
        i_crop_right    : in  std_logic_vector(10 downto 0);
        i_crop_top      : in  std_logic_vector(10 downto 0);
        i_crop_bottom   : in  std_logic_vector(10 downto 0);
        -- AXI4 read (ID set by the arbiter)
        m_araddr        : out std_logic_vector(31 downto 0);
        m_arlen         : out std_logic_vector(7 downto 0);
        m_arvalid       : out std_logic;
        m_arready       : in  std_logic;
        m_rdata         : in  std_logic_vector(63 downto 0);
        m_rlast         : in  std_logic;
        m_rvalid        : in  std_logic;
        m_rready        : out std_logic;
        -- line slots
        o_we            : out std_logic_vector(2 downto 0);    -- Y, Cb, Cr
        o_waddr         : out unsigned(9 downto 0);
        o_wdata         : out std_logic_vector(63 downto 0);
        o_meta          : out meta_arr_t;
        o_fetch_cnt     : out unsigned(15 downto 0);           -- lines published
        i_disp_cnt      : in  unsigned(15 downto 0);           -- lines shown (synchronised)
        o_frames        : out unsigned(15 downto 0);           -- frame switches
        o_busy          : out std_logic                        -- frames waiting to be shown
    );
end entity;

architecture rtl of h264_disp_rd is
    constant QN     : natural := 4;          -- decoded frames waiting
    constant DQN    : natural := 16;         -- burst descriptors
    constant MAXOUT : natural := 8;          -- bursts in flight
    constant BURST  : natural := 16;         -- beats per burst
    constant REFRESH : natural := 6000;      -- display refresh * 100

    type addr_arr_t is array (natural range <>) of unsigned(31 downto 0);
    type idx_arr_t is array (natural range <>) of unsigned(4 downto 0);

    signal q_idx : idx_arr_t(0 to QN - 1);
    signal q_y, q_cb, q_cr : addr_arr_t(0 to QN - 1);
    signal q_cnt : natural range 0 to QN;
    signal have_cur : std_logic;
    signal cur_idx : unsigned(4 downto 0);
    signal cur_y, cur_cb, cur_cr : unsigned(31 downto 0);
    signal acc : unsigned(15 downto 0);
    signal pend_rel : std_logic;             -- previous frame to release with the next descriptor
    signal pend_idx : unsigned(4 downto 0);
    signal frames : unsigned(15 downto 0);

    -- geometry of the current frame on the display
    signal x_start, x_src, y_start, y_src : unsigned(10 downto 0);
    signal vis_w, vis_h, y_end : unsigned(11 downto 0);
    signal bt709 : std_logic;
    signal g_sw, g_sw2, g_sh : unsigned(11 downto 0);    -- geometry steps
    signal rate_eff : unsigned(15 downto 0);        -- clamped frame rate
    signal g_c0 : unsigned(10 downto 0);
    signal base_y, base_cb, base_cr : unsigned(31 downto 0);   -- frame plane + x_src
    signal l_in : boolean;                          -- line with picture data
    signal l_sy : unsigned(10 downto 0);            -- its source line

    type st_t is (S_IDLE, S_FRAME, S_GEOM, S_GEOM1, S_GEOM2, S_LINE, S_LINE2, S_AR);
    signal st : st_t;
    signal row : unsigned(10 downto 0);
    signal iss_cnt : unsigned(15 downto 0);
    signal meta : meta_arr_t;

    type wl_arr_t is array (0 to 2) of unsigned(8 downto 0);
    type dst_arr_t is array (0 to 2) of unsigned(9 downto 0);
    signal pa : addr_arr_t(0 to 2);
    signal pwl : wl_arr_t;
    signal pdst : dst_arr_t;
    signal pl : natural range 0 to 2;
    signal arvalid : std_logic;
    signal ar_n : unsigned(4 downto 0);
    signal ar_lastp, ar_eol : std_logic;            -- burst ends the plane / the line

    -- descriptor: 24 marker, 23..22 plane, 21..12 dest, 11..7 beats, 6 eol, 5 rel, 4..0 rel index
    subtype desc_t is std_logic_vector(24 downto 0);
    type dq_t is array (0 to DQN - 1) of desc_t;
    signal dq : dq_t;
    signal dq_wr, dq_rd : unsigned(3 downto 0);
    signal dq_cnt : natural range 0 to DQN;
    signal outst : natural range 0 to MAXOUT;
    signal beat : unsigned(4 downto 0);
    signal fetch_cnt : unsigned(15 downto 0);
    signal rready : std_logic;

    function desc(marker : std_logic; plane : natural; dest : unsigned(9 downto 0); n : unsigned(4 downto 0);
                  eol, rel : std_logic; ridx : unsigned(4 downto 0)) return desc_t is
    begin
        return marker & std_logic_vector(to_unsigned(plane, 2)) & std_logic_vector(dest) &
               std_logic_vector(n) & eol & rel & std_logic_vector(ridx);
    end function;
begin
    o_frame_ready <= '1' when q_cnt < QN else '0';
    o_busy <= '1' when q_cnt > 0 else '0';
    m_arvalid <= arvalid;
    rready <= '1' when dq_cnt > 0 and dq(to_integer(dq_rd))(24) = '0' else '0';
    m_rready <= rready;
    o_meta <= meta;
    o_fetch_cnt <= fetch_cnt;
    o_frames <= frames;

    process (clk)
        variable push, pop, publish, relreq, flush_rel : boolean;
        variable pdesc, h : desc_t;
        variable rel_idx : unsigned(4 downto 0);
        variable diff : signed(15 downto 0);
        variable show : std_logic;
        variable rate, acc_n : unsigned(15 downto 0);
        variable adv, inside : boolean;
        variable qc : natural range 0 to QN;
        variable qpop, qpush : boolean;
        variable c0 : unsigned(10 downto 0);
        variable n : unsigned(8 downto 0);
        variable slot : natural range 0 to DISP_SLOTS - 1;
        variable m : meta_t;
    begin
        if rising_edge(clk) then
            push := false;
            pop := false;
            publish := false;
            relreq := false;
            qpop := false;
            o_we <= "000";
            o_release <= '0';
            show := i_show and i_lines_en;
            rate := unsigned(i_rate);
            if rate = 0 then
                rate_eff <= to_unsigned(3000, 16);
            elsif rate > REFRESH then
                rate_eff <= to_unsigned(REFRESH, 16);
            else
                rate_eff <= rate;
            end if;
            diff := signed(iss_cnt - i_disp_cnt);
            slot := to_integer(iss_cnt(1 downto 0));     -- = line count mod 4, as on the pixel side

            -- ---------------------------------------------------- issue side
            case st is
                when S_IDLE =>
                    if i_lines_en = '1' and diff < DISP_SLOTS and dq_cnt < DQN then
                        if row = 0 then
                            st <= S_FRAME;
                        else
                            st <= S_LINE;
                        end if;
                    end if;

                when S_FRAME =>
                    -- frame rate pacing: advance when the stream time passes a frame
                    acc_n := acc + rate_eff;
                    adv := acc_n >= REFRESH;
                    if adv then
                        acc <= acc_n - REFRESH;
                    else
                        acc <= acc_n;
                    end if;
                    if show = '1' and q_cnt > 0 and (have_cur = '0' or adv) then
                        if have_cur = '1' then
                            pend_rel <= '1';
                            pend_idx <= cur_idx;
                        end if;
                        cur_idx <= q_idx(0);
                        cur_y <= q_y(0);
                        cur_cb <= q_cb(0);
                        cur_cr <= q_cr(0);
                        have_cur <= '1';
                        qpop := true;
                        frames <= frames + 1;
                    end if;
                    st <= S_GEOM;

                when S_GEOM =>
                    -- visible size; a left crop is rounded down to 16 (shows up to 15 more columns)
                    g_sw <= unsigned(i_width) - unsigned(i_crop_right);
                    g_c0 <= unsigned(i_crop_left(10 downto 4)) & "0000";
                    g_sh <= unsigned(i_height) - unsigned(i_crop_top);
                    st <= S_GEOM1;

                when S_GEOM1 =>
                    g_sw2 <= g_sw - resize(g_c0, 12);
                    g_sh <= g_sh - unsigned(i_crop_bottom);
                    st <= S_GEOM2;

                when S_GEOM2 =>
                    if g_sw2 <= G_DISP_W then
                        x_start <= resize(shift_right(to_unsigned(G_DISP_W, 12) - g_sw2, 5) & "0000", 11);
                        x_src <= g_c0;
                        vis_w <= g_sw2;
                        base_y <= cur_y + g_c0;
                        base_cb <= cur_cb + g_c0(10 downto 1);
                        base_cr <= cur_cr + g_c0(10 downto 1);
                    else
                        c0 := g_c0 + resize(shift_right(g_sw2 - G_DISP_W, 5) & "0000", 11);
                        x_start <= (others => '0');
                        x_src <= c0;
                        vis_w <= to_unsigned(G_DISP_W, 12);
                        base_y <= cur_y + c0;
                        base_cb <= cur_cb + c0(10 downto 1);
                        base_cr <= cur_cr + c0(10 downto 1);
                    end if;
                    if g_sh <= G_DISP_H then
                        y_start <= resize(shift_right(to_unsigned(G_DISP_H, 12) - g_sh, 1), 11);
                        y_end <= resize(shift_right(to_unsigned(G_DISP_H, 12) - g_sh, 1), 12) + g_sh;
                        y_src <= unsigned(i_crop_top);
                        vis_h <= g_sh;
                    else
                        y_start <= (others => '0');
                        y_end <= to_unsigned(G_DISP_H, 12);
                        y_src <= unsigned(i_crop_top) + resize(shift_right(g_sh - G_DISP_H, 2) & "0", 11);
                        vis_h <= to_unsigned(G_DISP_H, 12);
                    end if;
                    if g_sh >= 720 then
                        bt709 <= '1';
                    else
                        bt709 <= '0';
                    end if;
                    st <= S_LINE;

                when S_LINE =>
                    -- picture data on this line?  (or blank / bars, or the display is ahead)
                    inside := have_cur = '1' and show = '1' and i_pattern = '0' and diff >= 0 and
                              vis_w /= 0 and row >= y_start and resize(row, 12) < y_end;
                    l_in <= inside;
                    l_sy <= y_src + (row - y_start);
                    m := (others => '0');
                    if have_cur = '0' or show = '0' or i_pattern = '1' then
                        m(25) := '1';
                    elsif not inside then
                        m(24) := '1';
                    end if;
                    m(23) := bt709;
                    m(22 downto 12) := std_logic_vector(x_start);
                    m(11 downto 0) := std_logic_vector(vis_w);
                    meta(slot) <= m;
                    st <= S_LINE2;

                when S_LINE2 =>
                    if not l_in then
                        -- publish without data
                        push := true;
                        pdesc := desc('1', 0, (others => '0'), (others => '0'), '1', pend_rel, pend_idx);
                        pend_rel <= '0';
                        iss_cnt <= iss_cnt + 1;
                        if row = G_DISP_H - 1 then
                            row <= (others => '0');
                        else
                            row <= row + 1;
                        end if;
                        st <= S_IDLE;
                    else
                        pa(0) <= base_y + shift_left(resize(l_sy, 32), 11);
                        pa(1) <= base_cb + shift_left(resize(l_sy(10 downto 1), 32), 10);
                        pa(2) <= base_cr + shift_left(resize(l_sy(10 downto 1), 32), 10);
                        pwl(0) <= resize(shift_right(vis_w + 7, 3), 9);
                        pwl(1) <= resize(shift_right(vis_w + 15, 4), 9);
                        pwl(2) <= resize(shift_right(vis_w + 15, 4), 9);
                        pdst(0) <= to_unsigned(slot * 256, 10);
                        pdst(1) <= to_unsigned(slot * 128, 10);
                        pdst(2) <= to_unsigned(slot * 128, 10);
                        pl <= 0;
                        st <= S_AR;
                    end if;

                when S_AR =>
                    if arvalid = '0' then
                        if pwl(pl) = 0 then
                            -- (a plane can only be empty if vis_w was cut to 0)
                            if pl = 2 then
                                st <= S_IDLE;
                            else
                                pl <= pl + 1;
                            end if;
                        elsif i_lines_en = '0' then
                            st <= S_IDLE;            -- abandon; counters restart after the drain
                        elsif dq_cnt < DQN and outst < MAXOUT then
                            if pwl(pl) > BURST then
                                n := to_unsigned(BURST, 9);
                                ar_lastp <= '0';
                                ar_eol <= '0';
                            else
                                n := pwl(pl);
                                ar_lastp <= '1';
                                if pl = 2 then
                                    ar_eol <= '1';
                                else
                                    ar_eol <= '0';
                                end if;
                            end if;
                            m_araddr <= std_logic_vector(pa(pl));
                            m_arlen <= std_logic_vector(resize(n - 1, 8));
                            ar_n <= n(4 downto 0);
                            arvalid <= '1';
                        end if;
                    elsif m_arready = '1' then
                        arvalid <= '0';
                        push := true;
                        pdesc := desc('0', pl, pdst(pl), ar_n, ar_eol, pend_rel, pend_idx);
                        pend_rel <= '0';
                        pa(pl) <= pa(pl) + shift_left(resize(ar_n, 32), 3);
                        pdst(pl) <= pdst(pl) + ar_n;
                        pwl(pl) <= pwl(pl) - ar_n;
                        if ar_eol = '1' then
                            iss_cnt <= iss_cnt + 1;
                            if row = G_DISP_H - 1 then
                                row <= (others => '0');
                            else
                                row <= row + 1;
                            end if;
                            st <= S_IDLE;
                        elsif ar_lastp = '1' then
                            pl <= pl + 1;
                        end if;
                    end if;
            end case;

            -- ------------------------------------------------------ data side
            if dq_cnt > 0 then
                h := dq(to_integer(dq_rd));
                if h(24) = '1' then
                    pop := true;
                elsif m_rvalid = '1' and rready = '1' then
                    o_we(to_integer(unsigned(h(23 downto 22)))) <= '1';
                    o_waddr <= unsigned(h(21 downto 12)) + beat;
                    o_wdata <= m_rdata;
                    if beat = unsigned(h(11 downto 7)) - 1 then
                        beat <= (others => '0');
                        pop := true;
                    else
                        beat <= beat + 1;
                    end if;
                end if;
                if pop then
                    publish := h(6) = '1';
                    relreq := h(5) = '1';
                    rel_idx := unsigned(h(4 downto 0));
                end if;
            end if;
            if publish then
                fetch_cnt <= fetch_cnt + 1;
            end if;

            -- descriptor FIFO and burst count
            if push then
                dq(to_integer(dq_wr)) <= pdesc;
                dq_wr <= dq_wr + 1;
            end if;
            if pop then
                dq_rd <= dq_rd + 1;
            end if;
            if push and not pop then
                dq_cnt <= dq_cnt + 1;
            elsif pop and not push then
                dq_cnt <= dq_cnt - 1;
            end if;
            if arvalid = '1' and m_arready = '1' then
                if not (pop and h(24) = '0') then
                    outst <= outst + 1;
                end if;
            elsif pop and h(24) = '0' then
                outst <= outst - 1;
            end if;

            -- ------------------------------------------- frame queue / release
            qc := q_cnt;
            flush_rel := false;
            if show = '0' and not relreq and not qpop then
                -- not displaying: hand every frame back
                if pend_rel = '1' then
                    flush_rel := true;
                    rel_idx := pend_idx;
                    pend_rel <= '0';
                elsif have_cur = '1' then
                    flush_rel := true;
                    rel_idx := cur_idx;
                    have_cur <= '0';
                elsif q_cnt > 0 then
                    flush_rel := true;
                    rel_idx := q_idx(0);
                    qpop := true;
                end if;
            end if;
            if relreq or flush_rel then
                o_release <= '1';
                o_release_index <= std_logic_vector(rel_idx);
            end if;
            qpush := i_frame_valid = '1' and q_cnt < QN;
            if qpop then
                for i in 0 to QN - 2 loop
                    q_idx(i) <= q_idx(i + 1);
                    q_y(i) <= q_y(i + 1);
                    q_cb(i) <= q_cb(i + 1);
                    q_cr(i) <= q_cr(i + 1);
                end loop;
                qc := qc - 1;
            end if;
            if qpush then
                q_idx(qc) <= unsigned(i_frame_index);
                q_y(qc) <= unsigned(i_frame_addr_y);
                q_cb(qc) <= unsigned(i_frame_addr_cb);
                q_cr(qc) <= unsigned(i_frame_addr_cr);
                qc := qc + 1;
            end if;
            q_cnt <= qc;

            -- pixel clock gone: drain, then restart the line counters
            if i_lines_en = '0' and st = S_IDLE and dq_cnt = 0 and outst = 0 and not push then
                row <= (others => '0');
                iss_cnt <= (others => '0');
                fetch_cnt <= (others => '0');
                beat <= (others => '0');
            end if;

            if i_flush = '1' then
                -- the decoder has been reset and owns all buffers again
                q_cnt <= 0;
                have_cur <= '0';
                pend_rel <= '0';
                o_release <= '0';
                for i in 0 to DQN - 1 loop
                    dq(i)(5) <= '0';
                end loop;
            end if;

            if rst = '1' then
                st <= S_IDLE;
                q_cnt <= 0;
                have_cur <= '0';
                pend_rel <= '0';
                acc <= (others => '0');
                frames <= (others => '0');
                row <= (others => '0');
                iss_cnt <= (others => '0');
                fetch_cnt <= (others => '0');
                arvalid <= '0';
                dq_wr <= (others => '0');
                dq_rd <= (others => '0');
                dq_cnt <= 0;
                outst <= 0;
                beat <= (others => '0');
                o_release <= '0';
                o_we <= "000";
                for i in 0 to DISP_SLOTS - 1 loop
                    meta(i) <= (25 => '1', others => '0');
                end loop;
                x_start <= (others => '0');
                vis_w <= (others => '0');
                bt709 <= '0';
            end if;
        end if;
    end process;
end architecture;

-- ============================================================================
-- pixel side (transceiver TX clock, 4 pixels per clock)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_disp_pkg.all;

entity h264_disp_px is
    generic (
        G_H_ACT  : positive := 1920;
        G_H_FP   : positive := 88;
        G_H_SYNC : positive := 44;
        G_H_BP   : positive := 148;
        G_V_ACT  : positive := 1080;
        G_V_FP   : positive := 4;
        G_V_SYNC : positive := 5;
        G_V_BP   : positive := 36;
        G_HS_POS : boolean := true;          -- sync polarity (CEA-861 HD: positive)
        G_VS_POS : boolean := true
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        i_bitrev    : in  std_logic;         -- reverse the 40 bit lane words
        o_raddr_y   : out unsigned(9 downto 0);
        o_raddr_c   : out unsigned(8 downto 0);
        i_rdata_y   : in  std_logic_vector(63 downto 0);
        i_rdata_cb  : in  std_logic_vector(63 downto 0);
        i_rdata_cr  : in  std_logic_vector(63 downto 0);
        i_meta      : in  meta_arr_t;        -- quasi static, valid for published slots
        i_fetch_cnt : in  unsigned(15 downto 0);  -- synchronised
        o_disp_cnt  : out unsigned(15 downto 0);
        o_underflow : out unsigned(15 downto 0);  -- lines shown black: data not there
        o_tmds_clk  : out std_logic_vector(39 downto 0);
        o_tmds_b    : out std_logic_vector(39 downto 0);
        o_tmds_g    : out std_logic_vector(39 downto 0);
        o_tmds_r    : out std_logic_vector(39 downto 0)
    );
end entity;

architecture rtl of h264_disp_px is
    constant HT  : natural := G_H_ACT + G_H_FP + G_H_SYNC + G_H_BP;
    constant VT  : natural := G_V_ACT + G_V_FP + G_V_SYNC + G_V_BP;
    constant HS0 : natural := G_H_ACT + G_H_FP;
    constant HS1 : natural := HS0 + G_H_SYNC;
    constant VS0 : natural := G_V_ACT + G_V_FP;
    constant VS1 : natural := VS0 + G_V_SYNC;
    constant GT  : natural := HT / 4;        -- clocks per line
    constant GA  : natural := G_H_ACT / 4;   -- active clocks per line
    constant BARW : natural := G_H_ACT / 8;  -- colour bar width

    type u8x4_t is array (0 to 3) of unsigned(7 downto 0);
    type s21x4_t is array (0 to 3) of signed(21 downto 0);

    signal g    : natural range 0 to GT - 1;
    signal v    : natural range 0 to VT - 1;
    signal disp_cnt : unsigned(15 downto 0);
    signal under : unsigned(15 downto 0);

    -- line mode (latched outside the active part of a line)
    signal l_pic, l_pat, l_709 : std_logic;
    signal l_xs  : unsigned(10 downto 0);
    signal l_xl  : unsigned(11 downto 0);
    signal l_rdy : std_logic;
    signal slot  : unsigned(1 downto 0);
    signal bar   : unsigned(2 downto 0);
    signal barc  : natural range 0 to BARW;

    -- pipeline
    constant NP : natural := 6;              -- control delay up to the TMDS input
    type ctl_t is record
        de, hs, vs, pic : std_logic_vector(3 downto 0);
        pat : std_logic;
        bar : unsigned(2 downto 0);
        sel : std_logic_vector(1 downto 0);  -- wx(3 downto 2): position inside the line words
        b709 : std_logic;
    end record;
    type ctl_arr_t is array (1 to NP) of ctl_t;
    signal ctl : ctl_arr_t;
    signal y3, cb3, cr3 : u8x4_t;
    signal t4, rc4, gb4, gr4, bc4 : s21x4_t;
    signal r6, g6, b6 : std_logic_vector(31 downto 0);
    signal de6, hs6, vs6 : std_logic_vector(3 downto 0);
    signal qb, qg, qr : std_logic_vector(39 downto 0);
    signal bitrev : std_logic;

    function clamp8(x : signed(21 downto 0)) return unsigned is
        variable r : signed(21 downto 0);
    begin
        r := shift_right(x + 512, 10);
        if r < 0 then
            return to_unsigned(0, 8);
        elsif r > 255 then
            return to_unsigned(255, 8);
        end if;
        return unsigned(r(7 downto 0));
    end function;

    function rev40(x : std_logic_vector(39 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(39 downto 0);
    begin
        for i in 0 to 39 loop
            r(i) := x(39 - i);
        end loop;
        return r;
    end function;
begin
    assert HT mod 4 = 0 and G_H_ACT mod 32 = 0
        report "h264_disp_px: horizontal total must be a multiple of 4, active width of 32" severity failure;

    o_disp_cnt <= disp_cnt;
    o_underflow <= under;

    process (clk)
        variable c : ctl_t;
        variable px, lv : natural;
        variable act : boolean;
        variable wx : signed(13 downto 0);
        variable m : meta_t;
        variable d : unsigned(15 downto 0);
        variable yw : std_logic_vector(31 downto 0);
        variable cbw, crw : std_logic_vector(15 downto 0);
        variable yy, cb, cr : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- ------------------------------------------------ timing (stage 0)
            act := v < G_V_ACT and g < GA;
            for i in 0 to 3 loop
                px := 4 * g + i;
                if act then
                    c.de(i) := '1';
                else
                    c.de(i) := '0';
                end if;
                if (px >= HS0 and px < HS1) = G_HS_POS then
                    c.hs(i) := '1';
                else
                    c.hs(i) := '0';
                end if;
                -- vertical sync edges on the horizontal sync start (CEA-861)
                if px >= HS0 then
                    lv := v;
                elsif v = 0 then
                    lv := VT - 1;
                else
                    lv := v - 1;
                end if;
                if (lv >= VS0 and lv < VS1) = G_VS_POS then
                    c.vs(i) := '1';
                else
                    c.vs(i) := '0';
                end if;
                wx := to_signed(px, 14) - signed(resize(l_xs, 14));
                if act and l_pic = '1' and wx >= 0 and wx < signed(resize(l_xl, 14)) then
                    c.pic(i) := '1';
                else
                    c.pic(i) := '0';
                end if;
            end loop;
            c.pat := l_pat;
            c.bar := bar;
            c.b709 := l_709;
            wx := to_signed(4 * g, 14) - signed(resize(l_xs, 14));
            c.sel := std_logic_vector(wx(3 downto 2));
            ctl(1) <= c;
            -- line slot addresses (data two clocks later, with ctl(2) -> ctl(3))
            o_raddr_y <= slot & unsigned(wx(10 downto 3));
            o_raddr_c <= slot & unsigned(wx(10 downto 4));
            for k in 2 to NP loop
                ctl(k) <= ctl(k - 1);
            end loop;

            -- counters
            if g = GT - 1 then
                g <= 0;
                if v = VT - 1 then
                    v <= 0;
                else
                    v <= v + 1;
                end if;
            else
                g <= g + 1;
            end if;
            if act and g = 0 and l_rdy = '0' and under /= x"FFFF" then
                under <= under + 1;
            end if;
            if v < G_V_ACT and g = GA then
                disp_cnt <= disp_cnt + 1;     -- slot read completely
            end if;
            -- colour bar position
            if g = GT - 1 then
                bar <= (others => '0');
                barc <= 0;
            elsif barc = BARW - 4 then
                barc <= 0;
                if bar /= 7 then
                    bar <= bar + 1;
                end if;
            else
                barc <= barc + 4;
            end if;
            -- next line's mode, while no active pixels are being read
            if not act then
                m := i_meta(to_integer(disp_cnt(1 downto 0)));
                d := i_fetch_cnt - disp_cnt;
                slot <= disp_cnt(1 downto 0);
                if d >= 1 and d <= DISP_SLOTS then
                    l_rdy <= '1';
                    l_pat <= m(25);
                    l_pic <= not m(25) and not m(24);
                else
                    l_rdy <= '0';
                    l_pat <= '0';
                    l_pic <= '0';
                end if;
                l_709 <= m(23);
                l_xs <= unsigned(m(22 downto 12));
                l_xl <= unsigned(m(11 downto 0));
            end if;

            -- ------------------------------------- stage 3: pick the samples
            -- a Y word holds 8 pixels, a Cb / Cr word the chroma of 16
            if ctl(2).sel(0) = '1' then
                yw := i_rdata_y(63 downto 32);
            else
                yw := i_rdata_y(31 downto 0);
            end if;
            case ctl(2).sel is
                when "00" => cbw := i_rdata_cb(15 downto 0); crw := i_rdata_cr(15 downto 0);
                when "01" => cbw := i_rdata_cb(31 downto 16); crw := i_rdata_cr(31 downto 16);
                when "10" => cbw := i_rdata_cb(47 downto 32); crw := i_rdata_cr(47 downto 32);
                when others => cbw := i_rdata_cb(63 downto 48); crw := i_rdata_cr(63 downto 48);
            end case;
            for j in 0 to 3 loop
                y3(j) <= unsigned(yw(8 * j + 7 downto 8 * j));
                cb3(j) <= unsigned(cbw(8 * (j / 2) + 7 downto 8 * (j / 2)));
                cr3(j) <= unsigned(crw(8 * (j / 2) + 7 downto 8 * (j / 2)));
            end loop;

            -- ------------------------------------------- stage 4: products
            for j in 0 to 3 loop
                yy := signed(resize(y3(j), 10)) - 16;
                cb := signed(resize(cb3(j), 10)) - 128;
                cr := signed(resize(cr3(j), 10)) - 128;
                t4(j) <= resize(yy * to_signed(1192, 13), 22);
                if ctl(3).b709 = '1' then
                    rc4(j) <= resize(cr * to_signed(1836, 13), 22);
                    gb4(j) <= resize(cb * to_signed(218, 13), 22);
                    gr4(j) <= resize(cr * to_signed(546, 13), 22);
                    bc4(j) <= resize(cb * to_signed(2163, 13), 22);
                else
                    rc4(j) <= resize(cr * to_signed(1634, 13), 22);
                    gb4(j) <= resize(cb * to_signed(401, 13), 22);
                    gr4(j) <= resize(cr * to_signed(833, 13), 22);
                    bc4(j) <= resize(cb * to_signed(2065, 13), 22);
                end if;
            end loop;

            -- -------------------------- stage 5/6: sums, clamp, final colour
            for j in 0 to 3 loop
                if ctl(4).pic(j) = '1' then
                    r6(8 * j + 7 downto 8 * j) <= std_logic_vector(clamp8(t4(j) + rc4(j)));
                    g6(8 * j + 7 downto 8 * j) <= std_logic_vector(clamp8(t4(j) - gb4(j) - gr4(j)));
                    b6(8 * j + 7 downto 8 * j) <= std_logic_vector(clamp8(t4(j) + bc4(j)));
                elsif ctl(4).pat = '1' and ctl(4).de(j) = '1' then
                    -- bars: white yellow cyan green magenta red blue black
                    r6(8 * j + 7 downto 8 * j) <= (others => '0');
                    g6(8 * j + 7 downto 8 * j) <= (others => '0');
                    b6(8 * j + 7 downto 8 * j) <= (others => '0');
                    if ctl(4).bar(1) = '0' then
                        r6(8 * j + 7 downto 8 * j) <= x"BF";
                    end if;
                    if ctl(4).bar(2) = '0' then
                        g6(8 * j + 7 downto 8 * j) <= x"BF";
                    end if;
                    if ctl(4).bar(0) = '0' then
                        b6(8 * j + 7 downto 8 * j) <= x"BF";
                    end if;
                else
                    r6(8 * j + 7 downto 8 * j) <= x"00";
                    g6(8 * j + 7 downto 8 * j) <= x"00";
                    b6(8 * j + 7 downto 8 * j) <= x"00";
                end if;
            end loop;
            de6 <= ctl(4).de;
            hs6 <= ctl(4).hs;
            vs6 <= ctl(4).vs;

            -- output words (optionally bit reversed, for a MSB first serialiser)
            bitrev <= i_bitrev;
            if bitrev = '1' then
                o_tmds_b <= rev40(qb);
                o_tmds_g <= rev40(qg);
                o_tmds_r <= rev40(qr);
                o_tmds_clk <= rev40(x"07C1F07C1F");
            else
                o_tmds_b <= qb;
                o_tmds_g <= qg;
                o_tmds_r <= qr;
                o_tmds_clk <= x"07C1F07C1F";     -- 0000011111 per pixel
            end if;

            if rst = '1' then
                g <= 0;
                v <= 0;
                disp_cnt <= (others => '0');
                under <= (others => '0');
                l_rdy <= '0';
                l_pic <= '0';
                l_pat <= '0';
                bar <= (others => '0');
                barc <= 0;
            end if;
        end if;
    end process;

    -- TMDS channel 0 (blue) carries HSYNC / VSYNC in the blanking
    u_tb : entity work.h264_tmds4
        port map (clk => clk, rst => rst, i_de => de6, i_c0 => hs6, i_c1 => vs6, i_d => b6, o_q => qb);
    u_tg : entity work.h264_tmds4
        port map (clk => clk, rst => rst, i_de => de6, i_c0 => "0000", i_c1 => "0000", i_d => g6, o_q => qg);
    u_tr : entity work.h264_tmds4
        port map (clk => clk, rst => rst, i_de => de6, i_c0 => "0000", i_c1 => "0000", i_d => r6, o_q => qr);
end architecture;

-- ============================================================================
-- wrapper: line slot RAMs and clock domain crossings
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_disp_pkg.all;

entity h264_disp is
    generic (
        G_H_ACT  : positive := 1920;
        G_H_FP   : positive := 88;
        G_H_SYNC : positive := 44;
        G_H_BP   : positive := 148;
        G_V_ACT  : positive := 1080;
        G_V_FP   : positive := 4;
        G_V_SYNC : positive := 5;
        G_V_BP   : positive := 36
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;
        -- control / status (decoder clock): ctrl 0 off, 1 colour bars,
        -- 2 bit reversed lanes, 31..16 fps * 100; status 15..0 frame
        -- switches, 31..16 lines shown black because data was late
        i_ctrl          : in  std_logic_vector(31 downto 0);
        o_status        : out std_logic_vector(31 downto 0);
        i_dec_rst       : in  std_logic := '0';   -- decoder reset: drop the frames it handed over
        -- decoded frames
        i_frame_valid   : in  std_logic;
        i_frame_index   : in  std_logic_vector(4 downto 0);
        i_frame_addr_y  : in  std_logic_vector(31 downto 0);
        i_frame_addr_cb : in  std_logic_vector(31 downto 0);
        i_frame_addr_cr : in  std_logic_vector(31 downto 0);
        o_frame_ready   : out std_logic;
        o_release       : out std_logic;
        o_release_index : out std_logic_vector(4 downto 0);
        i_width         : in  std_logic_vector(11 downto 0);
        i_height        : in  std_logic_vector(11 downto 0);
        i_crop_left     : in  std_logic_vector(10 downto 0);
        i_crop_right    : in  std_logic_vector(10 downto 0);
        i_crop_top      : in  std_logic_vector(10 downto 0);
        i_crop_bottom   : in  std_logic_vector(10 downto 0);
        -- AXI4 read
        m_araddr        : out std_logic_vector(31 downto 0);
        m_arlen         : out std_logic_vector(7 downto 0);
        m_arvalid       : out std_logic;
        m_arready       : in  std_logic;
        m_rdata         : in  std_logic_vector(63 downto 0);
        m_rlast         : in  std_logic;
        m_rvalid        : in  std_logic;
        m_rready        : out std_logic;
        -- transceiver side
        px_clk          : in  std_logic;    -- LANE0_TX_CLK_R (pixel clock / 4)
        px_clk_stable   : in  std_logic;    -- LANE0_TX_CLK_STABLE (asynchronous)
        o_tmds_clk      : out std_logic_vector(39 downto 0);
        o_tmds_b        : out std_logic_vector(39 downto 0);
        o_tmds_g        : out std_logic_vector(39 downto 0);
        o_tmds_r        : out std_logic_vector(39 downto 0);
        o_busy          : out std_logic         -- decoded frames still waiting for the screen
    );
end entity;

architecture rtl of h264_disp is
    signal we : std_logic_vector(2 downto 0);
    signal waddr : unsigned(9 downto 0);
    signal wdata : std_logic_vector(63 downto 0);
    signal raddr_y : unsigned(9 downto 0);
    signal raddr_c : unsigned(8 downto 0);
    signal rdata_y, rdata_cb, rdata_cr : std_logic_vector(63 downto 0);
    signal meta : meta_arr_t;
    signal fetch_cnt, disp_cnt, under, frames : unsigned(15 downto 0);

    -- clock domain crossings
    signal stable_s : std_logic_vector(1 downto 0);          -- -> clk
    signal lines_en : std_logic;
    signal px_rst_s : std_logic_vector(2 downto 0);          -- -> px_clk
    signal px_rst   : std_logic;
    signal fetch_g, disp_g, under_g : std_logic_vector(15 downto 0);
    signal fetch_s1, fetch_s2 : std_logic_vector(15 downto 0);  -- -> px_clk
    signal disp_s1, disp_s2   : std_logic_vector(15 downto 0);  -- -> clk
    signal under_s1, under_s2 : std_logic_vector(15 downto 0);  -- -> clk
    signal bitrev_s : std_logic_vector(1 downto 0);          -- -> px_clk
    signal fetch_px, disp_clk : unsigned(15 downto 0);
    signal show : std_logic;

    attribute syn_preserve : boolean;
    attribute syn_preserve of stable_s, px_rst_s, fetch_s1, fetch_s2, disp_s1, disp_s2,
        under_s1, under_s2, bitrev_s : signal is true;

    function bin2gray(b : unsigned(15 downto 0)) return std_logic_vector is
    begin
        return std_logic_vector(b xor shift_right(b, 1));
    end function;

    function gray2bin(g : std_logic_vector(15 downto 0)) return unsigned is
        variable b : unsigned(15 downto 0);
    begin
        b(15) := g(15);
        for i in 14 downto 0 loop
            b(i) := b(i + 1) xor g(i);
        end loop;
        return b;
    end function;
begin
    -- pixel clock present (decoder clock domain)
    process (clk)
    begin
        if rising_edge(clk) then
            stable_s <= stable_s(0) & px_clk_stable;
            disp_s1 <= disp_g;
            disp_s2 <= disp_s1;
            under_s1 <= under_g;
            under_s2 <= under_s1;
            if rst = '1' then
                stable_s <= "00";
            end if;
        end if;
    end process;
    lines_en <= stable_s(1);
    disp_clk <= gray2bin(disp_s2);
    o_status <= std_logic_vector(gray2bin(under_s2)) & std_logic_vector(frames);

    -- pixel domain reset follows the decoder reset and lines_en
    process (px_clk)
    begin
        if rising_edge(px_clk) then
            px_rst_s <= px_rst_s(1 downto 0) & (rst or not lines_en);
            fetch_s1 <= fetch_g;
            fetch_s2 <= fetch_s1;
            bitrev_s <= bitrev_s(0) & i_ctrl(2);
        end if;
    end process;
    px_rst <= px_rst_s(2);
    fetch_px <= gray2bin(fetch_s2);

    process (clk)
    begin
        if rising_edge(clk) then
            fetch_g <= bin2gray(fetch_cnt);
        end if;
    end process;

    process (px_clk)
    begin
        if rising_edge(px_clk) then
            disp_g <= bin2gray(disp_cnt);
            under_g <= bin2gray(under);
        end if;
    end process;

    show <= not i_ctrl(0);
    u_rd : entity work.h264_disp_rd
        generic map (G_DISP_W => G_H_ACT, G_DISP_H => G_V_ACT)
        port map (
            clk => clk, rst => rst, i_lines_en => lines_en, i_flush => i_dec_rst, i_show => show,
            i_pattern => i_ctrl(1), i_rate => i_ctrl(31 downto 16),
            i_frame_valid => i_frame_valid, i_frame_index => i_frame_index,
            i_frame_addr_y => i_frame_addr_y, i_frame_addr_cb => i_frame_addr_cb,
            i_frame_addr_cr => i_frame_addr_cr, o_frame_ready => o_frame_ready,
            o_release => o_release, o_release_index => o_release_index,
            i_width => i_width, i_height => i_height, i_crop_left => i_crop_left,
            i_crop_right => i_crop_right, i_crop_top => i_crop_top, i_crop_bottom => i_crop_bottom,
            m_araddr => m_araddr, m_arlen => m_arlen, m_arvalid => m_arvalid, m_arready => m_arready,
            m_rdata => m_rdata, m_rlast => m_rlast, m_rvalid => m_rvalid, m_rready => m_rready,
            o_we => we, o_waddr => waddr, o_wdata => wdata, o_meta => meta,
            o_fetch_cnt => fetch_cnt, i_disp_cnt => disp_clk, o_frames => frames, o_busy => o_busy);

    u_ram_y : entity work.h264_ram_dc
        generic map (G_AW => 10, G_DW => 64)
        port map (wclk => clk, we => we(0), waddr => waddr, wdata => wdata,
                  rclk => px_clk, raddr => raddr_y, rdata => rdata_y);
    u_ram_cb : entity work.h264_ram_dc
        generic map (G_AW => 9, G_DW => 64)
        port map (wclk => clk, we => we(1), waddr => waddr(8 downto 0), wdata => wdata,
                  rclk => px_clk, raddr => raddr_c, rdata => rdata_cb);
    u_ram_cr : entity work.h264_ram_dc
        generic map (G_AW => 9, G_DW => 64)
        port map (wclk => clk, we => we(2), waddr => waddr(8 downto 0), wdata => wdata,
                  rclk => px_clk, raddr => raddr_c, rdata => rdata_cr);

    u_px : entity work.h264_disp_px
        generic map (G_H_ACT => G_H_ACT, G_H_FP => G_H_FP, G_H_SYNC => G_H_SYNC, G_H_BP => G_H_BP,
                     G_V_ACT => G_V_ACT, G_V_FP => G_V_FP, G_V_SYNC => G_V_SYNC, G_V_BP => G_V_BP)
        port map (
            clk => px_clk, rst => px_rst, i_bitrev => bitrev_s(1),
            o_raddr_y => raddr_y, o_raddr_c => raddr_c,
            i_rdata_y => rdata_y, i_rdata_cb => rdata_cb, i_rdata_cr => rdata_cr,
            i_meta => meta, i_fetch_cnt => fetch_px, o_disp_cnt => disp_cnt, o_underflow => under,
            o_tmds_clk => o_tmds_clk, o_tmds_b => o_tmds_b, o_tmds_g => o_tmds_g, o_tmds_r => o_tmds_r);
end architecture;
