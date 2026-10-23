-- ============================================================================
-- tb_disp.vhd
-- Display path test: Annex-B file -> h264_dec_top -> frame store model ->
-- h264_disp (through h264_axi_rd_arb, decoder and display share the read
-- port) -> TMDS lane words.  The TMDS symbols are decoded back here:
--   * clock lane must carry 0000011111 for every pixel
--   * every symbol must be a control token or a valid data symbol, the
--     running disparity of the data must stay bounded
--   * line / frame timing (active pixels, totals, sync widths) is checked
--   * every complete frame (active RGB) is appended to G_OUT;
--     scripts/disp_check.py compares them with the reference decoder
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.tb_util_pkg.all;
use work.h264_pkg.all;

entity tb_disp is
    generic (
        G_IN        : string  := "in.264";
        G_OUT       : string  := "disp.rgb";
        G_MAXW      : natural := 352;
        G_MAXH      : natural := 288;
        G_NFB       : natural := 18;
        G_RLAT      : natural := 12;
        G_STALL     : natural := 1;
        G_PX_NS     : real    := 25.0;      -- pixel clock / 4 period
        G_RATE      : natural := 6000;      -- stream fps * 100
        G_EXTRA     : natural := 6;         -- frames captured after the stream is done
        G_TIMEOUT_US : natural := 200000;
        G_RESTART   : natural := 0;         -- 1: decoder reset after the stream, then the stream again
        G_H_ACT     : positive := 256;
        G_H_FP      : positive := 16;
        G_H_SYNC    : positive := 16;
        G_H_BP      : positive := 32;
        G_V_ACT     : positive := 192;
        G_V_FP      : positive := 2;
        G_V_SYNC    : positive := 2;
        G_V_BP      : positive := 4
    );
end entity;

architecture sim of tb_disp is
    type byte_arr_t is array (natural range <>) of natural range 0 to 255;
    type byte_ptr_t is access byte_arr_t;

    type mem_pt is protected
        procedure init(nfb, maxw, maxh : natural);
        impure function rd(addr : natural) return natural;
        procedure wr(addr : natural; v : natural);
        impure function errors return natural;
    end protected;

    type mem_pt is protected body
        variable m : byte_ptr_t;
        variable nf, mw, mh : natural;
        variable err : natural := 0;

        procedure init(nfb, maxw, maxh : natural) is
        begin
            nf := nfb;
            mw := maxw;
            mh := maxh;
            m := new byte_arr_t(0 to nfb * 3 * maxw * maxh - 1);
        end procedure;

        impure function index(addr : natural) return integer is
            variable fb, off, pl, x, y, w, h : natural;
        begin
            fb := addr / 2 ** 22;
            off := addr mod 2 ** 22;
            if off < 16#240000# then
                pl := 0; y := off / 2048; x := off mod 2048; w := mw; h := mh;
            elsif off < 16#2D0000# then
                pl := 1; off := off - 16#240000#; y := off / 1024; x := off mod 1024; w := mw / 2; h := mh / 2;
            else
                pl := 2; off := off - 16#2D0000#; y := off / 1024; x := off mod 1024; w := mw / 2; h := mh / 2;
            end if;
            if fb >= nf or x >= w or y >= h then
                return -1;
            end if;
            return (fb * 3 + pl) * mw * mh + y * mw + x;
        end function;

        impure function rd(addr : natural) return natural is
            variable i : integer;
        begin
            i := index(addr);
            if i < 0 then
                return 0;
            end if;
            return m(i);
        end function;

        procedure wr(addr : natural; v : natural) is
            variable i : integer;
        begin
            i := index(addr);
            if i < 0 then
                err := err + 1;
            else
                m(i) := v;
            end if;
        end procedure;

        impure function errors return natural is
        begin
            return err;
        end function;
    end protected body;

    shared variable mem : mem_pt;

    constant HT : natural := G_H_ACT + G_H_FP + G_H_SYNC + G_H_BP;
    constant VT : natural := G_V_ACT + G_V_FP + G_V_SYNC + G_V_BP;
    constant DISP_FRAME : time := real(HT / 4 * VT) * G_PX_NS * 1 ns;   -- one display frame

    signal clk, px_clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal restart : std_logic := '0';      -- decoder reset only (new stream from the PC)
    signal dec_rst : std_logic;
    signal last_feed : boolean := false;
    signal disp_busy : std_logic;
    signal px_stable : std_logic := '0';
    signal cycle : natural := 0;
    signal lfsr : std_logic_vector(15 downto 0) := x"ACE1";
    signal finished : boolean := false;

    signal s_tdata : std_logic_vector(7 downto 0) := (others => '0');
    signal s_tvalid, s_tlast : std_logic := '0';
    signal s_tready : std_logic;

    signal awaddr : std_logic_vector(31 downto 0);
    signal awlen : std_logic_vector(7 downto 0);
    signal awvalid, wvalid, wlast, bready : std_logic;
    signal awready, wready, bvalid : std_logic := '0';
    signal wdata : std_logic_vector(63 downto 0);
    signal wstrb : std_logic_vector(7 downto 0);
    -- decoder read port, display read port, joined port
    signal d_araddr, v_araddr, araddr : std_logic_vector(31 downto 0);
    signal d_arlen, v_arlen, arlen : std_logic_vector(7 downto 0);
    signal d_arvalid, d_arready, v_arvalid, v_arready, arvalid : std_logic;
    signal arready : std_logic := '0';
    signal arid, rid : std_logic_vector(3 downto 0) := "0000";
    signal rdata : std_logic_vector(63 downto 0);
    signal rlast, rvalid : std_logic := '0';
    signal d_rvalid, v_rvalid, d_rready, v_rready, rready : std_logic;

    signal frame_valid, frame_ready, rel : std_logic;
    signal frame_index, rel_index : std_logic_vector(4 downto 0);
    signal addr_y, addr_cb, addr_cr : std_logic_vector(31 downto 0);
    signal width, height : std_logic_vector(11 downto 0);
    signal crop_l, crop_r, crop_t, crop_b : std_logic_vector(10 downto 0);
    signal dec_err, stream_done : std_logic;
    signal pic_count : std_logic_vector(31 downto 0);
    signal ctrl, status : std_logic_vector(31 downto 0);
    signal t_clk, t_b, t_g, t_r : std_logic_vector(39 downto 0);
begin
    clk <= not clk after 5 ns when not finished;
    px_clk <= not px_clk after (G_PX_NS / 2.0) * 1 ns when not finished;
    ctrl <= std_logic_vector(to_unsigned(G_RATE, 16)) & x"0000";

    process (clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
        end if;
    end process;

    dec_rst <= rst or restart;
    dut : entity work.h264_dec_top
        port map (
            clk => clk, rst => dec_rst,
            s_axis_tdata => s_tdata, s_axis_tvalid => s_tvalid, s_axis_tlast => s_tlast, s_axis_tready => s_tready,
            m_axi_awid => open, m_axi_awaddr => awaddr, m_axi_awlen => awlen, m_axi_awsize => open,
            m_axi_awburst => open, m_axi_awlock => open, m_axi_awcache => open, m_axi_awprot => open,
            m_axi_awqos => open, m_axi_awvalid => awvalid, m_axi_awready => awready,
            m_axi_wdata => wdata, m_axi_wstrb => wstrb, m_axi_wlast => wlast, m_axi_wvalid => wvalid,
            m_axi_wready => wready, m_axi_bid => "0000", m_axi_bresp => "00", m_axi_bvalid => bvalid,
            m_axi_bready => bready,
            m_axi_arid => open, m_axi_araddr => d_araddr, m_axi_arlen => d_arlen, m_axi_arsize => open,
            m_axi_arburst => open, m_axi_arlock => open, m_axi_arcache => open, m_axi_arprot => open,
            m_axi_arqos => open, m_axi_arvalid => d_arvalid, m_axi_arready => d_arready,
            m_axi_rid => "0000", m_axi_rdata => rdata, m_axi_rresp => "00", m_axi_rlast => rlast,
            m_axi_rvalid => d_rvalid, m_axi_rready => d_rready,
            o_frame_valid => frame_valid, o_frame_index => frame_index,
            o_frame_addr_y => addr_y, o_frame_addr_cb => addr_cb, o_frame_addr_cr => addr_cr,
            i_frame_ready => frame_ready, i_release => rel, i_release_index => rel_index,
            o_width => width, o_height => height, o_crop_left => crop_l, o_crop_right => crop_r,
            o_crop_top => crop_t, o_crop_bottom => crop_b,
            o_error => dec_err, o_stream_done => stream_done, o_pic_count => pic_count);

    u_arb : entity work.h264_axi_rd_arb
        port map (
            clk => clk, rst => rst,
            s0_araddr => d_araddr, s0_arlen => d_arlen, s0_arvalid => d_arvalid, s0_arready => d_arready,
            s0_rvalid => d_rvalid, s0_rready => d_rready,
            s1_araddr => v_araddr, s1_arlen => v_arlen, s1_arvalid => v_arvalid, s1_arready => v_arready,
            s1_rvalid => v_rvalid, s1_rready => v_rready,
            m_arid => arid, m_araddr => araddr, m_arlen => arlen, m_arvalid => arvalid, m_arready => arready,
            m_rid => rid, m_rvalid => rvalid, m_rready => rready);

    u_disp : entity work.h264_disp
        generic map (G_H_ACT => G_H_ACT, G_H_FP => G_H_FP, G_H_SYNC => G_H_SYNC, G_H_BP => G_H_BP,
                     G_V_ACT => G_V_ACT, G_V_FP => G_V_FP, G_V_SYNC => G_V_SYNC, G_V_BP => G_V_BP)
        port map (
            clk => clk, rst => rst, i_ctrl => ctrl, o_status => status, i_dec_rst => restart,
            i_frame_valid => frame_valid, i_frame_index => frame_index,
            i_frame_addr_y => addr_y, i_frame_addr_cb => addr_cb, i_frame_addr_cr => addr_cr,
            o_frame_ready => frame_ready, o_release => rel, o_release_index => rel_index,
            i_width => width, i_height => height, i_crop_left => crop_l, i_crop_right => crop_r,
            i_crop_top => crop_t, i_crop_bottom => crop_b,
            m_araddr => v_araddr, m_arlen => v_arlen, m_arvalid => v_arvalid, m_arready => v_arready,
            m_rdata => rdata, m_rlast => rlast, m_rvalid => v_rvalid, m_rready => v_rready,
            px_clk => px_clk, px_clk_stable => px_stable,
            o_tmds_clk => t_clk, o_tmds_b => t_b, o_tmds_g => t_g, o_tmds_r => t_r, o_busy => disp_busy);

    -- stream input
    feed : process
        file f : char_file_t;
        variable c : character;
        variable st : file_open_status;
    begin
        mem.init(G_NFB, G_MAXW, G_MAXH);
        wait until rising_edge(clk);
        wait until rising_edge(clk);
        rst <= '0';
        wait for 2 us;
        px_stable <= '1';
        wait for 900 us;                    -- colour bars until the first picture
        for pass in 0 to G_RESTART loop
            if pass = G_RESTART then
                last_feed <= true;
            end if;
            file_open(st, f, G_IN, read_mode);
            assert st = open_ok report "cannot open " & G_IN severity failure;
            wait until rising_edge(clk);
            while not endfile(f) loop
                read(f, c);
                s_tdata <= std_logic_vector(to_unsigned(character'pos(c), 8));
                s_tvalid <= '1';
                if endfile(f) then
                    s_tlast <= '1';
                end if;
                wait until rising_edge(clk) and s_tready = '1';
            end loop;
            s_tvalid <= '0';
            s_tlast <= '0';
            file_close(f);
            if pass < G_RESTART then
                -- like a new START from the PC: the decoder is reset mid-display
                -- (after the last frame has been on the screen, as the PC waits for "done")
                wait until stream_done = '1' and disp_busy = '0';
                wait for 2 * DISP_FRAME;
                wait until rising_edge(clk);
                restart <= '1';
                for i in 1 to 16 loop
                    wait until rising_edge(clk);
                end loop;
                restart <= '0';
                wait for 1 us;
            end if;
        end loop;
        wait;
    end process;

    -- AXI slave, read channel (in-order, IDs echoed)
    rd_proc : process (clk)
        type q_t is array (0 to 31) of natural;
        variable qa, ql, qt, qi : q_t;
        variable qh, qn : natural := 0;
        variable beat : natural := 0;
        variable a : natural;
        variable d : std_logic_vector(63 downto 0);
        variable stall : boolean;
    begin
        if rising_edge(clk) then
            if arvalid = '1' and arready = '1' then
                qa((qh + qn) mod 32) := to_integer(unsigned(araddr));
                ql((qh + qn) mod 32) := to_integer(unsigned(arlen));
                qt((qh + qn) mod 32) := cycle;
                qi((qh + qn) mod 32) := to_integer(unsigned(arid));
                qn := qn + 1;
            end if;
            stall := G_STALL /= 0 and lfsr(3 downto 1) = "000";
            arready <= to_sl(qn < 30 and not stall);
            if rvalid = '1' and rready = '1' then
                if beat = ql(qh) then
                    qh := (qh + 1) mod 32;
                    qn := qn - 1;
                    beat := 0;
                else
                    beat := beat + 1;
                end if;
            end if;
            stall := G_STALL /= 0 and lfsr(7 downto 6) = "00";
            if qn > 0 and cycle >= qt(qh) + G_RLAT and not stall then
                a := qa(qh) + beat * 8;
                for i in 0 to 7 loop
                    d(i * 8 + 7 downto i * 8) := std_logic_vector(to_unsigned(mem.rd(a + i), 8));
                end loop;
                rdata <= d;
                rid <= std_logic_vector(to_unsigned(qi(qh), 4));
                rlast <= to_sl(beat = ql(qh));
                rvalid <= '1';
            else
                rvalid <= '0';
                rlast <= '0';
            end if;
            if rst = '1' then
                qn := 0;
                qh := 0;
                beat := 0;
                rvalid <= '0';
            end if;
        end if;
    end process;

    -- AXI slave, write channel
    wr_proc : process (clk)
        type q_t is array (0 to 15) of natural;
        variable qa, ql : q_t;
        variable qh, qn : natural := 0;
        variable beat : natural := 0;
        variable nb : natural := 0;
        variable a : natural;
        variable stall : boolean;
    begin
        if rising_edge(clk) then
            if awvalid = '1' and awready = '1' then
                qa((qh + qn) mod 16) := to_integer(unsigned(awaddr));
                ql((qh + qn) mod 16) := to_integer(unsigned(awlen));
                qn := qn + 1;
            end if;
            if wvalid = '1' and wready = '1' then
                a := qa(qh) + beat * 8;
                for i in 0 to 7 loop
                    if wstrb(i) = '1' then
                        mem.wr(a + i, to_integer(unsigned(wdata(i * 8 + 7 downto i * 8))));
                    end if;
                end loop;
                if beat = ql(qh) then
                    qh := (qh + 1) mod 16;
                    qn := qn - 1;
                    beat := 0;
                    nb := nb + 1;
                else
                    beat := beat + 1;
                end if;
            end if;
            if bvalid = '1' and bready = '1' then
                nb := nb - 1;
            end if;
            stall := G_STALL /= 0 and lfsr(5 downto 4) = "11";
            awready <= to_sl(qn < 14 and not stall);
            stall := G_STALL /= 0 and lfsr(9 downto 8) = "11";
            wready <= to_sl(qn > 0 and not stall);
            bvalid <= to_sl(nb > 0 and lfsr(2) = '0');
            if rst = '1' then
                qn := 0;
                qh := 0;
                beat := 0;
                nb := 0;
                bvalid <= '0';
                awready <= '0';
                wready <= '0';
            end if;
        end if;
    end process;

    -- TMDS receiver model
    rx : process (px_clk)
        file of_f : char_file_t open write_mode is G_OUT;
        type frame_t is array (0 to G_H_ACT * G_V_ACT * 3 - 1) of natural range 0 to 255;
        variable fr : frame_t;
        variable sym : std_logic_vector(9 downto 0);
        variable dat : std_logic_vector(7 downto 0);
        variable ctl : natural range 0 to 4;           -- 4 = data symbol
        variable de, hs, vs, de_p, hs_p, vs_p : boolean := false;
        variable rgb : byte_arr_t(0 to 2);
        variable disp : integer_vector(0 to 2) := (0, 0, 0);
        variable started : boolean := false;
        variable nfr, nerr, nlines, npix, hcnt, lcnt, hsw, vsl : natural := 0;
        variable act_lines, line_len, frame_lines : natural := 0;
        variable done_seen : natural := 0;
        variable ones : integer;
        variable l : line;

        procedure err(s : string) is
        begin
            if nerr < 20 then
                report "TMDS: " & s severity error;
            end if;
            nerr := nerr + 1;
        end procedure;

        -- control token (0..3 = c1 c0) or 4 for a data symbol, plus the data byte
        procedure decode(s : std_logic_vector(9 downto 0); c : out natural; d : out std_logic_vector(7 downto 0)) is
            variable t : std_logic_vector(7 downto 0);
        begin
            c := 4;
            case s is
                when "1101010100" => c := 0;
                when "0010101011" => c := 1;
                when "0101010100" => c := 2;
                when "1010101011" => c := 3;
                when others => null;
            end case;
            if s(9) = '1' then
                t := not s(7 downto 0);
            else
                t := s(7 downto 0);
            end if;
            d(0) := t(0);
            for i in 1 to 7 loop
                if s(8) = '1' then
                    d(i) := t(i) xor t(i - 1);
                else
                    d(i) := t(i) xnor t(i - 1);
                end if;
            end loop;
        end procedure;
    begin
        if rising_edge(px_clk) and px_stable = '1' then
            if t_clk /= x"07C1F07C1F" and started then
                err("clock lane pattern " & hex(t_clk, 10));
            end if;
            for j in 0 to 3 loop
                -- channel 0 (blue): sync tokens
                sym := t_b(10 * j + 9 downto 10 * j);
                decode(sym, ctl, dat);
                de := ctl = 4;
                if not de then
                    hs := ctl mod 2 = 1;
                    vs := ctl >= 2;
                end if;
                rgb(2) := to_integer(unsigned(dat));
                for k in 0 to 1 loop
                    if k = 0 then
                        sym := t_g(10 * j + 9 downto 10 * j);
                    else
                        sym := t_r(10 * j + 9 downto 10 * j);
                    end if;
                    decode(sym, ctl, dat);
                    if started and ((ctl = 4) /= de or (ctl /= 4 and ctl /= 0)) then
                        err("channel " & itoa(k + 1) & " does not follow channel 0");
                    end if;
                    rgb(1 - k) := to_integer(unsigned(dat));
                end loop;
                -- running disparity of the data symbols (bounded by the encoding)
                if de then
                    for ch in 0 to 2 loop
                        if ch = 0 then
                            sym := t_b(10 * j + 9 downto 10 * j);
                        elsif ch = 1 then
                            sym := t_g(10 * j + 9 downto 10 * j);
                        else
                            sym := t_r(10 * j + 9 downto 10 * j);
                        end if;
                        ones := 0;
                        for i in 0 to 9 loop
                            if sym(i) = '1' then
                                ones := ones + 1;
                            end if;
                        end loop;
                        disp(ch) := disp(ch) + 2 * ones - 10;
                        if started and abs (disp(ch)) > 20 then
                            err("running disparity " & itoa(disp(ch)));
                        end if;
                    end loop;
                else
                    disp := (0, 0, 0);
                end if;

                -- timing
                hcnt := hcnt + 1;
                if hs and not hs_p then
                    if started and line_len /= 0 and hcnt /= HT then
                        err("line length " & itoa(hcnt));
                    end if;
                    line_len := hcnt;
                    hcnt := 0;
                    lcnt := lcnt + 1;
                    if npix /= 0 then
                        if started and npix /= G_H_ACT then
                            err("active pixels " & itoa(npix));
                        end if;
                        act_lines := act_lines + 1;
                    end if;
                    npix := 0;
                end if;
                if hs then
                    hsw := hsw + 1;
                elsif hs_p and started and hsw /= G_H_SYNC then
                    err("hsync width " & itoa(hsw));
                end if;
                if not hs then
                    hsw := 0;
                end if;
                if vs and not vs_p then
                    -- frame boundary: store the frame just completed
                    if started then
                        if act_lines /= G_V_ACT then
                            err("active lines " & itoa(act_lines));
                        end if;
                        if lcnt /= VT then
                            err("lines per frame " & itoa(lcnt));
                        end if;
                        for i in fr'range loop
                            write(of_f, character'val(fr(i)));
                        end loop;
                        nfr := nfr + 1;
                        if stream_done = '1' and last_feed then
                            done_seen := done_seen + 1;
                        end if;
                    end if;
                    started := true;
                    act_lines := 0;
                    lcnt := 0;
                    nlines := 0;
                end if;
                if de then
                    if npix < G_H_ACT and act_lines < G_V_ACT then
                        for c in 0 to 2 loop
                            fr((act_lines * G_H_ACT + npix) * 3 + c) := rgb(c);
                        end loop;
                    end if;
                    npix := npix + 1;
                end if;
                hs_p := hs;
                vs_p := vs;
                de_p := de;
            end loop;
            if done_seen > G_EXTRA and not finished then
                write(l, "display test: " & itoa(nfr) & " frames captured, " & itoa(nerr) & " TMDS errors, " &
                         itoa(to_integer(unsigned(status(15 downto 0)))) & " frame switches, " &
                         itoa(to_integer(unsigned(status(31 downto 16)))) & " late lines, decoder error=" &
                         std_logic'image(dec_err) & ", pictures " & itoa(to_integer(unsigned(pic_count))));
                writeline(output, l);
                finished <= true;
            end if;
        end if;
    end process;

    process
    begin
        wait until finished for G_TIMEOUT_US * 1 us;
        assert finished report "timeout" severity failure;
        wait;
    end process;
end architecture;
