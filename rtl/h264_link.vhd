-- ============================================================================
-- h264_link.vhd
-- Stream link from a PC to the decoder over any byte channel (UART here).
--
-- Host -> FPGA frame:
--   55 AA | type | seq | len (2, LE, <= 1024) | payload (len) | crc16 (2, LE)
--   crc16: CCITT (poly 1021, init FFFF, no reflection) over type .. payload
--   types: 01 DATA (bitstream bytes), 02 END (end of stream),
--          03 START (reset decoder and link, seq 0), 04 STATUS (request)
-- FPGA -> host response (16 bytes):
--   5A A5 | type | seq | free (2) | pictures (4) | flags | width/height (3) | crc16 (2)
--   types: 81 ACK (frame seq accepted), 82 NAK (seq = next expected frame),
--          83 STATUS (seq = last accepted frame)
--   free  : bytes the FIFO can still take (credit for further DATA frames)
--   flags : 0 decoder error, 1 stream done, 2 END received, 3 FIFO empty
--   width / height: 12 bit each, byte 11 = w[7:0], 12 = h[3:0] & w[11:8],
--   13 = h[11:4]
--
-- DATA payload is written to the FIFO tentatively and committed when the
-- CRC is good and seq is the expected one (else rolled back).  Frames with
-- an older seq (retransmissions) are acknowledged again without writing,
-- frames with a newer seq get one NAK.  The last committed byte is held
-- back until more data or END arrives, so that it can carry TLAST.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;

entity h264_link is
    generic (
        G_FIFO_AW : positive := 15         -- FIFO of 2**G_FIFO_AW bytes
    );
    port (
        clk           : in  std_logic;
        rst           : in  std_logic;
        -- byte channel
        i_rx_data     : in  std_logic_vector(7 downto 0);
        i_rx_valid    : in  std_logic;
        o_tx_data     : out std_logic_vector(7 downto 0);
        o_tx_valid    : out std_logic;
        i_tx_ready    : in  std_logic;
        -- bitstream to the decoder
        m_axis_tdata  : out std_logic_vector(7 downto 0);
        m_axis_tvalid : out std_logic;
        m_axis_tlast  : out std_logic;
        m_axis_tready : in  std_logic;
        o_dec_rst     : out std_logic;     -- decoder reset after START (16 cycles)
        -- decoder status for the responses
        i_pic_count   : in  std_logic_vector(31 downto 0);
        i_error       : in  std_logic;
        i_stream_done : in  std_logic;
        i_width       : in  std_logic_vector(11 downto 0);
        i_height      : in  std_logic_vector(11 downto 0);
        -- activity (one cycle pulses, e.g. for LEDs)
        o_frame_ok    : out std_logic;
        o_frame_bad   : out std_logic
    );
end entity;

architecture rtl of h264_link is
    constant DEPTH    : natural := 2 ** G_FIFO_AW;
    constant MAXLEN   : natural := 1024;
    constant T_DATA   : std_logic_vector(7 downto 0) := x"01";
    constant T_END    : std_logic_vector(7 downto 0) := x"02";
    constant T_START  : std_logic_vector(7 downto 0) := x"03";
    constant T_STATUS : std_logic_vector(7 downto 0) := x"04";
    constant R_ACK    : std_logic_vector(7 downto 0) := x"81";
    constant R_NAK    : std_logic_vector(7 downto 0) := x"82";
    constant R_STATUS : std_logic_vector(7 downto 0) := x"83";

    function inflight(a, b : std_logic) return natural is
    begin
        if a = '1' and b = '1' then
            return 2;
        elsif a = '1' or b = '1' then
            return 1;
        end if;
        return 0;
    end function;

    function crc16_upd(c : std_logic_vector(15 downto 0); d : std_logic_vector(7 downto 0))
        return std_logic_vector is
        variable r : std_logic_vector(15 downto 0);
        variable fb : std_logic;
    begin
        r := c;
        for i in 7 downto 0 loop
            fb := r(15) xor d(i);
            r := r(14 downto 0) & '0';
            if fb = '1' then
                r := r xor x"1021";
            end if;
        end loop;
        return r;
    end function;

    subtype ptr_t is unsigned(G_FIFO_AW downto 0);

    -- parser
    type pst_t is (P_S0, P_S1, P_TYPE, P_SEQ, P_L0, P_L1, P_PAY, P_C0, P_C1);
    signal pst      : pst_t;
    signal f_type   : std_logic_vector(7 downto 0);
    signal f_seq    : unsigned(7 downto 0);
    signal f_len    : unsigned(10 downto 0);
    signal f_cnt    : unsigned(10 downto 0);
    signal f_l0     : std_logic_vector(7 downto 0);
    signal f_c0     : std_logic_vector(7 downto 0);
    signal crc      : std_logic_vector(15 downto 0);
    signal accept   : std_logic;           -- DATA payload goes to the FIFO
    signal exp_seq  : unsigned(7 downto 0);
    signal nak_sent : std_logic;
    signal end_flag : std_logic;

    -- FIFO
    signal wr_tent, wr_cmt, rd_ptr : ptr_t;
    signal ram_we   : std_logic;
    signal ram_wa   : unsigned(G_FIFO_AW - 1 downto 0);
    signal ram_wd   : std_logic_vector(7 downto 0);
    signal ram_ra   : unsigned(G_FIFO_AW - 1 downto 0);
    signal ram_rd   : std_logic_vector(7 downto 0);
    signal free_b   : unsigned(G_FIFO_AW downto 0);
    -- read pipeline (2 cycle RAM latency) and output queue
    signal iv1, iv2 : std_logic;
    signal il1, il2 : std_logic;
    type oq_t is array (0 to 3) of std_logic_vector(8 downto 0);   -- last & data
    signal oq       : oq_t;
    signal oq_cnt   : natural range 0 to 4;
    signal dec_rst_cnt : natural range 0 to 16;

    -- responses
    signal pend_v    : std_logic;
    signal pend_type : std_logic_vector(7 downto 0);
    signal pend_seq  : unsigned(7 downto 0);
    signal tx_busy   : std_logic;
    signal tx_idx    : natural range 0 to 15;
    type tb_t is array (0 to 13) of std_logic_vector(7 downto 0);
    signal tx_buf    : tb_t;
    signal tx_crc    : std_logic_vector(15 downto 0);
    signal tx_v      : std_logic;
    signal tx_d      : std_logic_vector(7 downto 0);
begin
    u_ram : entity work.h264_ram_sdp
        generic map (G_AW => G_FIFO_AW, G_DW => 8)
        port map (clk => clk, we => ram_we, waddr => ram_wa, wdata => ram_wd, raddr => ram_ra, rdata => ram_rd);

    free_b <= to_unsigned(DEPTH - 1, G_FIFO_AW + 1) - (wr_cmt - rd_ptr);
    m_axis_tvalid <= '1' when oq_cnt /= 0 else '0';
    m_axis_tdata <= oq(0)(7 downto 0);
    m_axis_tlast <= oq(0)(8);
    o_dec_rst <= '1' when dec_rst_cnt /= 0 else '0';
    o_tx_valid <= tx_v;
    o_tx_data <= tx_d;

    process (clk)
        variable resp      : boolean;
        variable rtype     : std_logic_vector(7 downto 0);
        variable rseq      : unsigned(7 downto 0);
        variable len       : unsigned(10 downto 0);
        variable older     : boolean;
        variable d         : unsigned(7 downto 0);
        variable cnt       : ptr_t;
        variable hold      : ptr_t;
        variable issue     : boolean;
        variable pop       : boolean;
        variable q         : oq_t;
        variable qn        : natural range 0 to 5;
        variable flags     : std_logic_vector(7 downto 0);
        variable fifo_rst  : boolean;
    begin
        if rising_edge(clk) then
            ram_we <= '0';
            o_frame_ok <= '0';
            o_frame_bad <= '0';
            resp := false;
            rtype := R_ACK;
            rseq := (others => '0');
            fifo_rst := false;
            if dec_rst_cnt /= 0 then
                dec_rst_cnt <= dec_rst_cnt - 1;
            end if;

            -- --------------------------------------------------------
            -- frame parser
            -- --------------------------------------------------------
            if i_rx_valid = '1' then
                case pst is
                    when P_S0 =>
                        if i_rx_data = x"55" then
                            pst <= P_S1;
                        end if;
                    when P_S1 =>
                        if i_rx_data = x"AA" then
                            pst <= P_TYPE;
                            crc <= x"FFFF";
                        elsif i_rx_data /= x"55" then
                            pst <= P_S0;
                        end if;
                    when P_TYPE =>
                        f_type <= i_rx_data;
                        crc <= crc16_upd(crc, i_rx_data);
                        pst <= P_SEQ;
                    when P_SEQ =>
                        f_seq <= unsigned(i_rx_data);
                        crc <= crc16_upd(crc, i_rx_data);
                        pst <= P_L0;
                    when P_L0 =>
                        f_l0 <= i_rx_data;
                        crc <= crc16_upd(crc, i_rx_data);
                        pst <= P_L1;
                    when P_L1 =>
                        crc <= crc16_upd(crc, i_rx_data);
                        if unsigned(i_rx_data) > MAXLEN / 256 then
                            pst <= P_S0;            -- not a frame
                        else
                            len := unsigned(i_rx_data(2 downto 0)) & unsigned(f_l0);
                            if len > MAXLEN then
                                pst <= P_S0;
                            else
                                f_len <= len;
                                f_cnt <= (others => '0');
                                wr_tent <= wr_cmt;
                                if f_type = T_DATA and f_seq = exp_seq and end_flag = '0' and
                                   resize(len, G_FIFO_AW + 1) <= free_b then
                                    accept <= '1';
                                else
                                    accept <= '0';
                                end if;
                                if len = 0 then
                                    pst <= P_C0;
                                else
                                    pst <= P_PAY;
                                end if;
                            end if;
                        end if;
                    when P_PAY =>
                        crc <= crc16_upd(crc, i_rx_data);
                        if accept = '1' then
                            ram_we <= '1';
                            ram_wa <= wr_tent(G_FIFO_AW - 1 downto 0);
                            ram_wd <= i_rx_data;
                            wr_tent <= wr_tent + 1;
                        end if;
                        f_cnt <= f_cnt + 1;
                        if f_cnt = f_len - 1 then
                            pst <= P_C0;
                        end if;
                    when P_C0 =>
                        f_c0 <= i_rx_data;
                        pst <= P_C1;
                    when P_C1 =>
                        pst <= P_S0;
                        d := exp_seq - f_seq;
                        older := d >= 1 and d <= 128;
                        if (i_rx_data & f_c0) = crc then
                            o_frame_ok <= '1';
                            if f_type = T_DATA or f_type = T_END then
                                if f_seq = exp_seq and (f_type = T_END or accept = '1') then
                                    if f_type = T_DATA then
                                        wr_cmt <= wr_tent;
                                    else
                                        end_flag <= '1';
                                    end if;
                                    exp_seq <= exp_seq + 1;
                                    nak_sent <= '0';
                                    resp := true; rtype := R_ACK; rseq := f_seq;
                                elsif older then
                                    -- retransmission of an accepted frame
                                    resp := true; rtype := R_ACK; rseq := exp_seq - 1;
                                elsif f_seq = exp_seq then
                                    -- no room (or data after END): the host waits for credit
                                    resp := true; rtype := R_NAK; rseq := exp_seq;
                                elsif nak_sent = '0' then
                                    resp := true; rtype := R_NAK; rseq := exp_seq;
                                    nak_sent <= '1';
                                end if;
                            elsif f_type = T_START then
                                fifo_rst := true;
                                exp_seq <= f_seq + 1;
                                nak_sent <= '0';
                                end_flag <= '0';
                                dec_rst_cnt <= 16;
                                resp := true; rtype := R_ACK; rseq := f_seq;
                            elsif f_type = T_STATUS then
                                resp := true; rtype := R_STATUS; rseq := exp_seq - 1;
                            end if;
                        else
                            o_frame_bad <= '1';
                            if nak_sent = '0' then
                                resp := true; rtype := R_NAK; rseq := exp_seq;
                                nak_sent <= '1';
                            end if;
                        end if;
                end case;
            end if;

            -- --------------------------------------------------------
            -- FIFO read side: hold back the last byte until END
            -- --------------------------------------------------------
            cnt := wr_cmt - rd_ptr;
            if end_flag = '1' then
                hold := (others => '0');
            else
                hold := to_unsigned(1, G_FIFO_AW + 1);
            end if;
            q := oq;
            qn := oq_cnt;
            pop := qn /= 0 and m_axis_tready = '1';
            if pop then
                for i in 0 to 2 loop
                    q(i) := q(i + 1);
                end loop;
                qn := qn - 1;
            end if;
            issue := cnt > hold and oq_cnt + inflight(iv1, iv2) < 4;
            iv1 <= '0';
            if issue then
                ram_ra <= rd_ptr(G_FIFO_AW - 1 downto 0);
                rd_ptr <= rd_ptr + 1;
                iv1 <= '1';
                il1 <= end_flag and to_sl(cnt = 1);
            end if;
            iv2 <= iv1;
            il2 <= il1;
            if iv2 = '1' then
                q(qn) := il2 & ram_rd;
                qn := qn + 1;
            end if;
            oq <= q;
            oq_cnt <= qn;

            -- --------------------------------------------------------
            -- responses
            -- --------------------------------------------------------
            if resp then
                pend_v <= '1';
                pend_type <= rtype;
                pend_seq <= rseq;
            end if;
            if tx_v = '1' and i_tx_ready = '1' then
                tx_v <= '0';
            end if;
            if tx_busy = '0' then
                if pend_v = '1' and not resp then
                    -- snapshot of the status
                    flags := (others => '0');
                    flags(0) := i_error;
                    flags(1) := i_stream_done;
                    flags(2) := end_flag;
                    if wr_cmt = rd_ptr and oq_cnt = 0 and iv1 = '0' and iv2 = '0' then
                        flags(3) := '1';
                    end if;
                    tx_buf(0) <= x"5A";
                    tx_buf(1) <= x"A5";
                    tx_buf(2) <= pend_type;
                    tx_buf(3) <= std_logic_vector(pend_seq);
                    if free_b > 65535 then
                        tx_buf(4) <= x"FF";
                        tx_buf(5) <= x"FF";
                    else
                        tx_buf(4) <= std_logic_vector(resize(free_b, 16)(7 downto 0));
                        tx_buf(5) <= std_logic_vector(resize(free_b, 16)(15 downto 8));
                    end if;
                    tx_buf(6) <= i_pic_count(7 downto 0);
                    tx_buf(7) <= i_pic_count(15 downto 8);
                    tx_buf(8) <= i_pic_count(23 downto 16);
                    tx_buf(9) <= i_pic_count(31 downto 24);
                    tx_buf(10) <= flags;
                    tx_buf(11) <= i_width(7 downto 0);
                    tx_buf(12) <= i_height(3 downto 0) & i_width(11 downto 8);
                    tx_buf(13) <= i_height(11 downto 4);
                    pend_v <= '0';
                    tx_busy <= '1';
                    tx_idx <= 0;
                    tx_crc <= x"FFFF";
                end if;
            elsif tx_v = '0' or i_tx_ready = '1' then
                -- next byte (0..13 from the buffer, 14 / 15 = crc)
                tx_v <= '1';
                if tx_idx <= 13 then
                    tx_d <= tx_buf(tx_idx);
                    if tx_idx >= 2 then
                        tx_crc <= crc16_upd(tx_crc, tx_buf(tx_idx));
                    end if;
                elsif tx_idx = 14 then
                    tx_d <= tx_crc(7 downto 0);
                else
                    tx_d <= tx_crc(15 downto 8);
                end if;
                if tx_idx = 15 then
                    tx_busy <= '0';
                else
                    tx_idx <= tx_idx + 1;
                end if;
            end if;

            if fifo_rst then
                wr_cmt <= (others => '0');
                wr_tent <= (others => '0');
                rd_ptr <= (others => '0');
                iv1 <= '0';
                iv2 <= '0';
                oq_cnt <= 0;
            end if;
            if rst = '1' then
                pst <= P_S0;
                exp_seq <= (others => '0');
                nak_sent <= '0';
                end_flag <= '0';
                accept <= '0';
                wr_cmt <= (others => '0');
                wr_tent <= (others => '0');
                rd_ptr <= (others => '0');
                iv1 <= '0';
                iv2 <= '0';
                oq_cnt <= 0;
                pend_v <= '0';
                tx_busy <= '0';
                tx_v <= '0';
                dec_rst_cnt <= 16;
            end if;
        end if;
    end process;
end architecture;
