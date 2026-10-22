-- ============================================================================
-- tb_top.vhd
-- Full decoder testbench:
--   * feeds an Annex-B file over AXI4-Stream
--   * AXI4 slave memory model (frame store) with read latency, multiple
--     outstanding reads and optional random back-pressure
--   * every announced frame is cropped and appended to a YUV file, then
--     released back to the decoder
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.tb_util_pkg.all;
use work.h264_pkg.all;

entity tb_top is
    generic (
        G_IN         : string  := "in.264";
        G_OUT        : string  := "out.yuv";
        G_MAXW       : natural := 352;      -- largest coded width of the stream
        G_MAXH       : natural := 288;
        G_NFB        : natural := 18;
        G_RLAT       : natural := 12;       -- read latency in cycles
        G_STALL      : natural := 1;        -- random AXI back-pressure
        G_TIMEOUT_US : natural := 2000000;
        G_RTRACE     : string  := "";       -- unfiltered macroblock trace (debug)
        G_PERF       : natural := 1         -- report busy cycles per stage
    );
end entity;

architecture sim of tb_top is

    -- ------------------------------------------------------------------
    -- frame store model
    -- ------------------------------------------------------------------
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

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal done : boolean := false;

    signal s_tdata  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_tvalid : std_logic := '0';
    signal s_tlast  : std_logic := '0';
    signal s_tready : std_logic;

    signal awaddr, araddr : std_logic_vector(31 downto 0);
    signal awlen, arlen : std_logic_vector(7 downto 0);
    signal awvalid, awready, wvalid, wready, wlast, bvalid, bready : std_logic;
    signal arvalid, arready, rvalid, rready, rlast : std_logic;
    signal wdata, rdata : std_logic_vector(63 downto 0);
    signal wstrb : std_logic_vector(7 downto 0);

    signal frame_valid, frame_ready, rel : std_logic := '0';
    signal frame_index, rel_index : std_logic_vector(4 downto 0) := (others => '0');
    signal width, height : std_logic_vector(11 downto 0);
    signal crop_l, crop_r, crop_t, crop_b : std_logic_vector(10 downto 0);
    signal dec_err, stream_done : std_logic;
    signal pic_count : std_logic_vector(31 downto 0);
    signal cycle : natural := 0;
    signal lfsr : std_logic_vector(15 downto 0) := x"ACE1";
begin
    clk <= not clk after 5 ns when not done;

    dut : entity work.h264_dec_top
        port map (
            clk => clk, rst => rst,
            s_axis_tdata => s_tdata, s_axis_tvalid => s_tvalid, s_axis_tlast => s_tlast, s_axis_tready => s_tready,
            m_axi_awid => open, m_axi_awaddr => awaddr, m_axi_awlen => awlen, m_axi_awsize => open,
            m_axi_awburst => open, m_axi_awlock => open, m_axi_awcache => open, m_axi_awprot => open,
            m_axi_awqos => open, m_axi_awvalid => awvalid, m_axi_awready => awready,
            m_axi_wdata => wdata, m_axi_wstrb => wstrb, m_axi_wlast => wlast, m_axi_wvalid => wvalid,
            m_axi_wready => wready, m_axi_bid => "0000", m_axi_bresp => "00", m_axi_bvalid => bvalid,
            m_axi_bready => bready,
            m_axi_arid => open, m_axi_araddr => araddr, m_axi_arlen => arlen, m_axi_arsize => open,
            m_axi_arburst => open, m_axi_arlock => open, m_axi_arcache => open, m_axi_arprot => open,
            m_axi_arqos => open, m_axi_arvalid => arvalid, m_axi_arready => arready,
            m_axi_rid => "0000", m_axi_rdata => rdata, m_axi_rresp => "00", m_axi_rlast => rlast,
            m_axi_rvalid => rvalid, m_axi_rready => rready,
            o_frame_valid => frame_valid, o_frame_index => frame_index, o_frame_addr_y => open,
            o_frame_addr_cb => open, o_frame_addr_cr => open, i_frame_ready => frame_ready,
            i_release => rel, i_release_index => rel_index,
            o_width => width, o_height => height, o_crop_left => crop_l, o_crop_right => crop_r,
            o_crop_top => crop_t, o_crop_bottom => crop_b,
            o_error => dec_err, o_stream_done => stream_done, o_pic_count => pic_count);

    -- stream feeder
    feed : process
        file f : char_file_t;
        variable c : character;
        variable st : file_open_status;
    begin
        mem.init(G_NFB, G_MAXW, G_MAXH);
        file_open(st, f, G_IN, read_mode);
        assert st = open_ok report "cannot open " & G_IN severity failure;
        wait until rising_edge(clk);
        wait until rising_edge(clk);
        rst <= '0';
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
        wait;
    end process;

    process (clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
        end if;
    end process;

    -- ------------------------------------------------------------------
    -- AXI slave: read channel
    -- ------------------------------------------------------------------
    rd_proc : process (clk)
        type q_t is array (0 to 31) of natural;
        variable qa, ql, qt : q_t;
        variable qh, qn : natural := 0;
        variable beat : natural := 0;
        variable act : boolean := false;
        variable a : natural;
        variable d : std_logic_vector(63 downto 0);
        variable stall : boolean;
    begin
        if rising_edge(clk) then
            -- accept address
            if arvalid = '1' and arready = '1' then
                qa((qh + qn) mod 32) := to_integer(unsigned(araddr));
                ql((qh + qn) mod 32) := to_integer(unsigned(arlen));
                qt((qh + qn) mod 32) := cycle;
                qn := qn + 1;
            end if;
            stall := G_STALL /= 0 and lfsr(3 downto 1) = "000";
            if qn < 30 and not stall then
                arready <= '1';
            else
                arready <= '0';
            end if;
            -- data
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

    -- ------------------------------------------------------------------
    -- AXI slave: write channel
    -- ------------------------------------------------------------------
    wr_proc : process (clk)
        type q_t is array (0 to 15) of natural;
        variable qa, ql : q_t;
        variable qh, qn : natural := 0;
        variable beat : natural := 0;
        variable nb : natural := 0;          -- pending B responses
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
                assert (wlast = '1') = (beat = ql(qh)) report "WLAST mismatch" severity error;
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

    -- ------------------------------------------------------------------
    -- frame output
    -- ------------------------------------------------------------------
    frame_ready <= '1';
    out_proc : process (clk)
        file of_f : char_file_t open write_mode is G_OUT;
        variable base, x0, x1, y0, y1, fb, w, h : natural;
        variable nframes : natural := 0;
    begin
        if rising_edge(clk) then
            rel <= '0';
            if frame_valid = '1' and frame_ready = '1' then
                fb := to_integer(unsigned(frame_index));
                w := to_integer(unsigned(width));
                h := to_integer(unsigned(height));
                for pl in 0 to 2 loop
                    if pl = 0 then
                        x0 := to_integer(unsigned(crop_l)); x1 := w - to_integer(unsigned(crop_r));
                        y0 := to_integer(unsigned(crop_t)); y1 := h - to_integer(unsigned(crop_b));
                        base := fb * 2 ** 22;
                        for y in y0 to y1 - 1 loop
                            for x in x0 to x1 - 1 loop
                                write(of_f, character'val(mem.rd(base + y * 2048 + x)));
                            end loop;
                        end loop;
                    else
                        x0 := to_integer(unsigned(crop_l)) / 2; x1 := (w - to_integer(unsigned(crop_r))) / 2;
                        y0 := to_integer(unsigned(crop_t)) / 2; y1 := (h - to_integer(unsigned(crop_b))) / 2;
                        if pl = 1 then
                            base := fb * 2 ** 22 + 16#240000#;
                        else
                            base := fb * 2 ** 22 + 16#2D0000#;
                        end if;
                        for y in y0 to y1 - 1 loop
                            for x in x0 to x1 - 1 loop
                                write(of_f, character'val(mem.rd(base + y * 1024 + x)));
                            end loop;
                        end loop;
                    end if;
                end loop;
                nframes := nframes + 1;
                rel <= '1';
                rel_index <= frame_index;
            end if;
            if stream_done = '1' and not done then
                report "stream done: " & integer'image(nframes) & " frames, " & integer'image(cycle) &
                       " cycles, error=" & std_logic'image(dec_err) & " mem errors=" & integer'image(mem.errors);
                done <= true;
            end if;
        end if;
    end process;

    process
    begin
        wait until done for G_TIMEOUT_US * 1 us;
        if not done then
            report "timeout" severity failure;
        end if;
        wait;
    end process;


    -- busy cycles of every pipeline stage (state /= idle)
    gen_perf : if G_PERF /= 0 generate
        alias st_mbp is << signal .tb_top.dut.u_mbp.st_dbg : natural >>;
        alias st_mc  is << signal .tb_top.dut.u_mc.st_dbg  : natural >>;
        alias st_rc  is << signal .tb_top.dut.u_rec.st_dbg : natural >>;
        alias st_db  is << signal .tb_top.dut.u_db.st_dbg  : natural >>;
    begin
        process (clk)
            variable b_mbp, b_mc, b_rc, b_db : natural := 0;
            variable reported : boolean := false;
            type hist_t is array (0 to 63) of natural;
            variable h_mbp, h_mc, h_rc, h_db : hist_t := (others => 0);
            procedure hist_report(name : string; h : hist_t) is
            begin
                for i in 0 to 63 loop
                    if h(i) /= 0 then
                        report name & " state " & integer'image(i) & ": " & integer'image(h(i));
                    end if;
                end loop;
            end procedure;
        begin
            if rising_edge(clk) then
                -- per state cycle histograms (G_PERF > 1)
                h_mbp(st_mbp) := h_mbp(st_mbp) + 1;
                h_mc(st_mc) := h_mc(st_mc) + 1;
                h_rc(st_rc) := h_rc(st_rc) + 1;
                h_db(st_db) := h_db(st_db) + 1;
                if stream_done = '1' and not reported and G_PERF > 1 then
                    hist_report("mbparse", h_mbp);
                    hist_report("mc", h_mc);
                    hist_report("recon", h_rc);
                    hist_report("deblock", h_db);
                end if;
                if st_mbp /= 0 then b_mbp := b_mbp + 1; end if;
                if st_mc /= 0 then b_mc := b_mc + 1; end if;
                if st_rc /= 0 then b_rc := b_rc + 1; end if;
                if st_db /= 0 then b_db := b_db + 1; end if;
                if stream_done = '1' and not reported then
                    reported := true;
                    report "busy cycles: mbparse " & integer'image(b_mbp) & " mc " & integer'image(b_mc) &
                           " recon " & integer'image(b_rc) & " deblock " & integer'image(b_db);
                end if;
            end if;
        end process;
    end generate;

    -- optional trace of the reconstructed (unfiltered) macroblocks
    gen_rtrace : if G_RTRACE'length > 0 generate
        alias rw_en   is << signal .tb_top.dut.rw_en   : std_logic >>;
        alias rw_addr is << signal .tb_top.dut.rw_addr : unsigned(5 downto 0) >>;
        alias rw_data is << signal .tb_top.dut.rw_data : std_logic_vector(127 downto 0) >>;
        alias t_r     is << signal .tb_top.dut.t_r     : mbinfo_t >>;
        alias t_r_v   is << signal .tb_top.dut.t_r_v   : std_logic >>;
        alias db_rdy  is << signal .tb_top.dut.db_rdy  : std_logic >>;
    begin
        process (clk)
            -- block addressed buffer: (slot*32 + block, sample)
            type buf_t is array (0 to 63, 0 to 15) of natural range 0 to 255;
            variable b : buf_t;
            file tf : text open write_mode is G_RTRACE;
            variable l : line;
            variable s0 : natural;
        begin
            if rising_edge(clk) then
                if rw_en = '1' then
                    for i in 0 to 15 loop
                        b(to_integer(rw_addr), i) := to_integer(unsigned(rw_data(i * 8 + 7 downto i * 8)));
                    end loop;
                end if;
                if t_r_v = '1' and db_rdy = '1' then
                    s0 := 0;
                    if t_r.rec_slot = '1' then
                        s0 := 32;
                    end if;
                    write(l, "R " & integer'image(to_integer(t_r.mb_y) * to_integer(t_r.pic_w) + to_integer(t_r.mb_x)));
                    for y in 0 to 15 loop
                        for x in 0 to 15 loop
                            write(l, " " & integer'image(b(s0 + (y / 4) * 4 + x / 4, (y mod 4) * 4 + x mod 4)));
                        end loop;
                    end loop;
                    for c in 0 to 1 loop
                        for y in 0 to 7 loop
                            for x in 0 to 7 loop
                                write(l, " " & integer'image(b(s0 + 16 + c * 4 + (y / 4) * 2 + x / 4, (y mod 4) * 4 + x mod 4)));
                            end loop;
                        end loop;
                    end loop;
                    writeline(tf, l);
                end if;
            end if;
        end process;
    end generate;
end architecture;
