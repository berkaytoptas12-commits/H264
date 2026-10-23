-- ============================================================================
-- tb_cosim.vhd
-- Whole chain co-simulation: the kit firmware (kit/sim/kit_cosim, the real
-- bridge / network code with fabric_mmio.c) reaches the complete fabric
-- design h264_dec_hdmi_top through its APB port; requests come over pipes
-- (kit/sim/cosim_vhpi.c, VHPIDIRECT), so a PC program sending a video over
-- UDP drives h264_apb_bridge -> h264_link -> h264_dec_top -> DDR model ->
-- h264_disp -> TMDS.  The TMDS lanes are decoded back as in tb_disp and the
-- frames are written to G_OUT (compare with scripts/disp_check.py).  The
-- simulation ends when the firmware closes the pipe.
--   LD_LIBRARY_PATH must contain kit/sim (libh264cosim.so).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.tb_util_pkg.all;
use work.h264_pkg.all;

entity tb_cosim is
    generic (
        G_OUT        : string  := "cosim.rgb";
        G_MAXW       : natural := 352;
        G_MAXH       : natural := 288;
        G_NFB        : natural := 18;
        G_RLAT       : natural := 12;
        G_STALL      : natural := 1;
        G_PX_NS      : real    := 25.0;
        G_APB_GAP    : natural := 8;        -- idle clocks after an APB access (CPU / FIC3)
        G_IDLE       : natural := 200;      -- clocks simulated while no request is waiting
        G_TIMEOUT_US : natural := 2000000;
        G_H_ACT      : positive := 256;
        G_H_FP       : positive := 16;
        G_H_SYNC     : positive := 16;
        G_H_BP       : positive := 32;
        G_V_ACT      : positive := 192;
        G_V_FP       : positive := 2;
        G_V_SYNC     : positive := 2;
        G_V_BP       : positive := 4
    );
end entity;

architecture sim of tb_cosim is
    function cs_poll(d : integer) return integer;
    attribute foreign of cs_poll : function is "VHPIDIRECT libh264cosim.so cs_poll";
    function cs_addr(d : integer) return integer;
    attribute foreign of cs_addr : function is "VHPIDIRECT libh264cosim.so cs_addr";
    function cs_hi(d : integer) return integer;
    attribute foreign of cs_hi : function is "VHPIDIRECT libh264cosim.so cs_hi";
    function cs_lo(d : integer) return integer;
    attribute foreign of cs_lo : function is "VHPIDIRECT libh264cosim.so cs_lo";
    function cs_reply(hi, lo : integer) return integer;
    attribute foreign of cs_reply : function is "VHPIDIRECT libh264cosim.so cs_reply";

    function cs_poll(d : integer) return integer is
    begin
        report "libh264cosim.so not loaded" severity failure;
        return 3;
    end function;
    function cs_addr(d : integer) return integer is begin return 0; end function;
    function cs_hi(d : integer) return integer is begin return 0; end function;
    function cs_lo(d : integer) return integer is begin return 0; end function;
    function cs_reply(hi, lo : integer) return integer is begin return 0; end function;

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

    signal clk, px_clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal px_stable : std_logic := '0';
    signal cycle : natural := 0;
    signal lfsr : std_logic_vector(15 downto 0) := x"ACE1";
    signal finished, quit : boolean := false;
    signal napb : natural := 0;

    signal paddr, pwdata, prdata : std_logic_vector(31 downto 0) := (others => '0');
    signal psel, penable, pwrite, pready, pslverr : std_logic := '0';
    signal led_rx, led_err, led_done : std_logic;

    signal awaddr : std_logic_vector(31 downto 0);
    signal awlen : std_logic_vector(7 downto 0);
    signal awvalid, wvalid, wlast, bready : std_logic;
    signal awready, wready, bvalid : std_logic := '0';
    signal wdata : std_logic_vector(63 downto 0);
    signal wstrb : std_logic_vector(7 downto 0);
    signal araddr : std_logic_vector(31 downto 0);
    signal arlen : std_logic_vector(7 downto 0);
    signal arvalid : std_logic;
    signal arready : std_logic := '0';
    signal arid, rid : std_logic_vector(3 downto 0) := "0000";
    signal rdata : std_logic_vector(63 downto 0);
    signal rlast, rvalid : std_logic := '0';
    signal rready : std_logic;
    signal t_clk, t_b, t_g, t_r : std_logic_vector(39 downto 0);
begin
    clk <= not clk after 5 ns when not finished;
    px_clk <= not px_clk after (G_PX_NS / 2.0) * 1 ns when not finished;

    process (clk)
    begin
        if rising_edge(clk) then
            cycle <= cycle + 1;
            lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
        end if;
    end process;

    dut : entity work.h264_dec_hdmi_top
        generic map (G_H_ACT => G_H_ACT, G_H_FP => G_H_FP, G_H_SYNC => G_H_SYNC, G_H_BP => G_H_BP,
                     G_V_ACT => G_V_ACT, G_V_FP => G_V_FP, G_V_SYNC => G_V_SYNC, G_V_BP => G_V_BP)
        port map (
            clk => clk, rst => rst,
            paddr => paddr, psel => psel, penable => penable, pwrite => pwrite, pwdata => pwdata,
            prdata => prdata, pready => pready, pslverr => pslverr,
            led_rx => led_rx, led_err => led_err, led_done => led_done,
            m_axi_awid => open, m_axi_awaddr => awaddr, m_axi_awlen => awlen, m_axi_awsize => open,
            m_axi_awburst => open, m_axi_awlock => open, m_axi_awcache => open, m_axi_awprot => open,
            m_axi_awqos => open, m_axi_awvalid => awvalid, m_axi_awready => awready,
            m_axi_wdata => wdata, m_axi_wstrb => wstrb, m_axi_wlast => wlast, m_axi_wvalid => wvalid,
            m_axi_wready => wready, m_axi_bid => "0000", m_axi_bresp => "00", m_axi_bvalid => bvalid,
            m_axi_bready => bready,
            m_axi_arid => arid, m_axi_araddr => araddr, m_axi_arlen => arlen, m_axi_arsize => open,
            m_axi_arburst => open, m_axi_arlock => open, m_axi_arcache => open, m_axi_arprot => open,
            m_axi_arqos => open, m_axi_arvalid => arvalid, m_axi_arready => arready,
            m_axi_rid => rid, m_axi_rdata => rdata, m_axi_rresp => "00", m_axi_rlast => rlast,
            m_axi_rvalid => rvalid, m_axi_rready => rready,
            hdmi_clk => px_clk, hdmi_clk_stable => px_stable,
            o_tmds_clk => t_clk, o_tmds_b => t_b, o_tmds_g => t_g, o_tmds_r => t_r);

    -- APB master: one request from the firmware at a time
    apb : process
        variable op, a, hi, lo, r : integer;
    begin
        mem.init(G_NFB, G_MAXW, G_MAXH);
        for i in 1 to 10 loop
            wait until rising_edge(clk);
        end loop;
        rst <= '0';
        wait for 2 us;
        px_stable <= '1';
        loop
            op := cs_poll(0);
            if op = 3 then
                quit <= true;
                wait;
            elsif op = 0 then
                for i in 1 to G_IDLE loop
                    wait until rising_edge(clk);
                end loop;
            else
                a := cs_addr(0);
                paddr <= std_logic_vector(to_unsigned(a, 32));
                pwrite <= to_sl(op = 2);
                if op = 2 then
                    hi := cs_hi(0);
                    lo := cs_lo(0);
                    pwdata <= std_logic_vector(to_unsigned(hi, 16)) & std_logic_vector(to_unsigned(lo, 16));
                end if;
                psel <= '1';
                wait until rising_edge(clk);
                penable <= '1';
                wait until rising_edge(clk) and pready = '1';
                if op = 1 then
                    r := cs_reply(to_integer(unsigned(prdata(31 downto 16))), to_integer(unsigned(prdata(15 downto 0))));
                end if;
                psel <= '0';
                penable <= '0';
                pwrite <= '0';
                napb <= napb + 1;
                for i in 1 to G_APB_GAP loop
                    wait until rising_edge(clk);
                end loop;
            end if;
        end loop;
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
            if quit and not finished then
                write(l, "co-simulation: " & itoa(nfr) & " frames captured, " & itoa(nerr) & " TMDS errors, " &
                         itoa(napb) & " APB accesses");
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
