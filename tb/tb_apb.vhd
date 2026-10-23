-- ============================================================================
-- tb_apb.vhd
-- MSS path test: an APB master (stands for the firmware on the MSS) writes
-- the link frames built by host/make_vectors.py into h264_apb_bridge
-- (TXDATA4 / TXDATA), reads the responses (RXDATA) and drives h264_link
-- like the firmware + PC program would: credit waits, a corrupted frame
-- (NAK, resend), a repeated frame, END, CTRL reset.  The AXI-Stream output
-- must be the original file with TLAST on the last byte.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_apb is
    generic (
        G_IN     : string := "in.264";
        G_FRAMES : string := "frames.txt";
        G_RESP   : string := "responses.bin";
        G_AW     : positive := 11               -- link FIFO 2 KiB: forces credit waits
    );
end entity;

architecture sim of tb_apb is
    constant TCLK : time := 10 ns;
    signal clk     : std_logic := '0';
    signal rst     : std_logic := '1';
    signal paddr   : std_logic_vector(11 downto 0) := (others => '0');
    signal psel, penable, pwrite : std_logic := '0';
    signal pwdata, prdata : std_logic_vector(31 downto 0) := (others => '0');
    signal pready, pslverr : std_logic;
    signal rx_data, tx_data : std_logic_vector(7 downto 0);
    signal rx_valid, tx_valid, tx_ready : std_logic;
    signal tdata   : std_logic_vector(7 downto 0);
    signal tvalid, tlast, tready : std_logic;
    signal dec_rst, soft_rst, link_rst : std_logic;
    signal done_s  : std_logic := '0';
    signal lfsr    : std_logic_vector(15 downto 0) := x"ACE1";
    signal stall   : boolean := true;
    signal sink_n  : natural := 0;
    signal sink_end : boolean := false;
    signal finished : boolean := false;
    signal sink_rst : boolean := false;
    type char_file_t is file of character;

    function crc16_upd(c : natural; d : natural) return natural is
        variable r : unsigned(15 downto 0);
        variable dv : unsigned(7 downto 0);
        variable fb : std_logic;
    begin
        r := to_unsigned(c, 16);
        dv := to_unsigned(d, 8);
        for i in 7 downto 0 loop
            fb := r(15) xor dv(i);
            r := r(14 downto 0) & '0';
            if fb = '1' then
                r := r xor x"1021";
            end if;
        end loop;
        return to_integer(r);
    end function;
begin
    clk <= not clk after TCLK / 2 when not finished;

    u_apb : entity work.h264_apb_bridge
        port map (clk => clk, rst => rst, paddr => paddr, psel => psel, penable => penable,
                  pwrite => pwrite, pwdata => pwdata, prdata => prdata, pready => pready, pslverr => pslverr,
                  o_rx_data => rx_data, o_rx_valid => rx_valid,
                  i_tx_data => tx_data, i_tx_valid => tx_valid, o_tx_ready => tx_ready,
                  i_pic_count => x"00000007", i_error => '0', i_stream_done => done_s,
                  i_width => x"280", i_height => x"1E0", o_soft_rst => soft_rst);
    link_rst <= rst or soft_rst;
    dut : entity work.h264_link
        generic map (G_FIFO_AW => G_AW)
        port map (clk => clk, rst => link_rst, i_rx_data => rx_data, i_rx_valid => rx_valid,
                  o_tx_data => tx_data, o_tx_valid => tx_valid, i_tx_ready => tx_ready,
                  m_axis_tdata => tdata, m_axis_tvalid => tvalid, m_axis_tlast => tlast,
                  m_axis_tready => tready, o_dec_rst => dec_rst,
                  i_pic_count => x"00000007", i_error => '0', i_stream_done => done_s,
                  i_width => x"280", i_height => x"1E0", o_frame_ok => open, o_frame_bad => open);

    process (clk)
    begin
        if rising_edge(clk) then
            lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
        end if;
    end process;
    tready <= '0' when stall else lfsr(0) or lfsr(5);

    sink : process
        file f : char_file_t;
        variable c : character;
        type mem_t is array (0 to 1048575) of natural range 0 to 255;
        variable mem : mem_t;
        variable n, k : natural := 0;
        variable st : file_open_status;
    begin
        file_open(st, f, G_IN, read_mode);
        assert st = open_ok report "cannot open " & G_IN severity failure;
        while not endfile(f) loop
            read(f, c);
            mem(n) := character'pos(c);
            n := n + 1;
        end loop;
        file_close(f);
        loop
            wait until rising_edge(clk);
            if tvalid = '1' and tready = '1' then
                assert k < n report "more bytes than in the file" severity failure;
                assert to_integer(unsigned(tdata)) = mem(k)
                    report "byte " & integer'image(k) & " differs" severity failure;
                assert (tlast = '1') = (k = n - 1)
                    report "TLAST wrong at byte " & integer'image(k) severity failure;
                k := k + 1;
                sink_n <= k;
                if tlast = '1' then
                    sink_end <= true;
                    done_s <= '1';
                end if;
            end if;
        end loop;
    end process;

    host : process
        file ff : text;
        file fo : char_file_t;
        variable l : line;
        type bytes_t is array (0 to 2097151) of natural range 0 to 255;
        variable fb : bytes_t;
        type idx_t is array (0 to 8191) of natural;
        variable fo_i, fl : idx_t;
        variable nf, pos, v : natural := 0;
        variable ch : character;
        variable rdv : std_logic_vector(31 downto 0);
        variable resp : integer_vector(0 to 15);
        variable r_type, r_seq, r_free, r_flags, r_pics, r_w, r_h : natural;
        variable seq, plen, j, polls, credit_waits, ndata : natural := 0;
        variable st : file_open_status;

        function hexval(h : character) return natural is
        begin
            if h >= '0' and h <= '9' then
                return character'pos(h) - character'pos('0');
            end if;
            return character'pos(h) - character'pos('a') + 10;
        end function;

        procedure apb_write(a : natural; d : std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clk);
            paddr <= std_logic_vector(to_unsigned(a, 12));
            pwdata <= d;
            pwrite <= '1';
            psel <= '1';
            penable <= '0';
            wait until rising_edge(clk);
            penable <= '1';
            wait until rising_edge(clk) and pready = '1';
            psel <= '0';
            penable <= '0';
            pwrite <= '0';
        end procedure;

        procedure apb_read(a : natural; d : out std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clk);
            paddr <= std_logic_vector(to_unsigned(a, 12));
            pwrite <= '0';
            psel <= '1';
            penable <= '0';
            wait until rising_edge(clk);
            penable <= '1';
            wait until rising_edge(clk) and pready = '1';
            d := prdata;
            psel <= '0';
            penable <= '0';
        end procedure;

        -- the firmware's job: copy one datagram (frame) into the bridge
        procedure send_frame(k : natural; corrupt : integer) is
            variable n, i, b : natural;
            variable w : std_logic_vector(31 downto 0);
            variable need : natural;
        begin
            n := fl(k);
            need := (n + 3) / 4;
            loop
                apb_read(16#08#, w);
                exit when to_integer(unsigned(w)) >= need;
            end loop;
            i := 0;
            while i < n loop
                if n - i >= 4 then
                    for m in 0 to 3 loop
                        b := fb(fo_i(k) + i + m);
                        if i + m = corrupt then
                            b := (b + 64) mod 256;
                        end if;
                        w(m * 8 + 7 downto m * 8) := std_logic_vector(to_unsigned(b, 8));
                    end loop;
                    apb_write(16#10#, w);
                    i := i + 4;
                else
                    b := fb(fo_i(k) + i);
                    if i = corrupt then
                        b := (b + 64) mod 256;
                    end if;
                    apb_write(16#0C#, x"000000" & std_logic_vector(to_unsigned(b, 8)));
                    i := i + 1;
                end if;
            end loop;
        end procedure;

        procedure get_response(what : string) is
            variable cnt : natural := 0;
            variable crc : natural;
            variable tries : natural := 0;
        begin
            while cnt < 16 loop
                apb_read(16#18#, rdv);
                if rdv(8) = '1' then
                    v := to_integer(unsigned(rdv(7 downto 0)));
                    write(fo, character'val(v));
                    if (cnt = 0 and v /= 16#5A#) or (cnt = 1 and v /= 16#A5#) then
                        cnt := 0;
                    else
                        resp(cnt) := v;
                        cnt := cnt + 1;
                    end if;
                else
                    tries := tries + 1;
                    assert tries < 100000 report "no response (" & what & ")" severity failure;
                end if;
            end loop;
            crc := 16#FFFF#;
            for i in 2 to 13 loop
                crc := crc16_upd(crc, resp(i));
            end loop;
            assert crc = resp(14) + 256 * resp(15) report "response CRC wrong" severity failure;
            r_type := resp(2);
            r_seq := resp(3);
            r_free := resp(4) + 256 * resp(5);
            r_pics := resp(6);
            r_flags := resp(10);
            r_w := resp(11) + 256 * (resp(12) mod 16);
            r_h := resp(12) / 16 + 16 * resp(13);
        end procedure;

        procedure expect(t, s : natural; what : string) is
        begin
            get_response(what);
            assert r_type = t and r_seq = s
                report what & ": got type " & integer'image(r_type) & " seq " & integer'image(r_seq) &
                       ", expected " & integer'image(t) & " / " & integer'image(s) severity failure;
        end procedure;

        variable kq, kend : natural;
    begin
        file_open(st, ff, G_FRAMES, read_mode);
        assert st = open_ok report "cannot open " & G_FRAMES severity failure;
        file_open(st, fo, G_RESP, write_mode);
        while not endfile(ff) loop
            readline(ff, l);
            exit when l'length < 3;
            read(l, ch);
            fo_i(nf) := pos;
            while l'length > 1 loop
                read(l, ch);
                v := hexval(ch) * 16;
                read(l, ch);
                fb(pos) := v + hexval(ch);
                pos := pos + 1;
            end loop;
            fl(nf) := pos - fo_i(nf);
            nf := nf + 1;
        end loop;
        kq := 1;
        kend := nf - 1;

        wait for 100 ns;
        rst <= '0';
        wait for 500 ns;
        apb_read(16#00#, rdv);
        assert rdv = x"48323634" report "ID register" severity failure;
        apb_read(16#04#, rdv);
        assert to_integer(unsigned(rdv(15 downto 0))) = 512 report "CAPS register" severity failure;
        apb_read(16#24#, rdv);
        assert unsigned(rdv(11 downto 0)) = 640 and unsigned(rdv(27 downto 16)) = 480 report "SIZE register" severity failure;

        send_frame(0, -1);
        expect(16#81#, 0, "START");
        send_frame(kq, -1);
        expect(16#83#, 0, "STATUS");
        assert r_pics = 7 and r_w = 640 and r_h = 480 report "status fields" severity failure;

        for k in 2 to kend - 1 loop
            seq := (k - 1) mod 256;
            plen := fb(fo_i(k) + 4) + 256 * fb(fo_i(k) + 5);
            j := k - 2;
            polls := 0;
            while plen > r_free loop
                wait for 20 us;
                send_frame(kq, -1);
                expect(16#83#, (seq - 1) mod 256, "STATUS while waiting for credit");
                polls := polls + 1;
                credit_waits := credit_waits + 1;
                if polls = 4 then
                    stall <= false;
                end if;
            end loop;
            if j mod 4 = 2 then
                send_frame(k, 6 + plen / 2);
                expect(16#82#, seq, "NAK after CRC error");
            end if;
            send_frame(k, -1);
            expect(16#81#, seq, "ACK");
            if j mod 5 = 3 then
                send_frame(k, -1);
                expect(16#81#, seq, "ACK for a repeated frame");
            end if;
            ndata := ndata + 1;
        end loop;
        stall <= false;
        send_frame(kend, -1);
        expect(16#81#, (kend - 1) mod 256, "END");
        wait until sink_end for 10 ms;
        assert sink_end report "TLAST not received" severity failure;
        wait for 1 us;
        send_frame(kq, -1);
        expect(16#83#, (kend - 1) mod 256, "STATUS at the end");
        assert r_flags = 16#0E# report "flags at the end" severity failure;
        apb_read(16#1C#, rdv);
        assert rdv(1) = '1' report "STATUS register: stream done" severity failure;

        -- CTRL reset: link and decoder reset, FIFOs empty
        apb_write(16#28#, x"00000001");
        wait until rising_edge(clk);
        assert soft_rst = '1' report "no reset from CTRL" severity failure;
        wait for 1 us;
        apb_read(16#14#, rdv);
        assert unsigned(rdv) = 0 report "RX FIFO not empty after reset" severity failure;
        send_frame(0, -1);
        expect(16#81#, 0, "START after CTRL reset");
        assert r_free = 2 ** G_AW - 1 report "FIFO not empty after reset" severity failure;

        assert credit_waits > 0 report "credit wait not exercised" severity failure;
        report "APB path test passed: " & integer'image(ndata) & " data frames, " &
               integer'image(sink_n) & " bytes, last byte with TLAST, " &
               integer'image(credit_waits) & " credit polls";
        finished <= true;
        wait;
    end process;
end architecture;
