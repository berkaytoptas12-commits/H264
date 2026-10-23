-- ============================================================================
-- tb_link.vhd
-- Stream link test: frames built by host/h264link.py (host/make_vectors.py)
-- are sent over the UART to h264_uart_rx -> h264_link; the AXI-Stream
-- output (random back-pressure, long initial stall) must be the original
-- file with TLAST on its last byte.  Directed cases: corrupted frame (NAK,
-- resend), frame out of order (NAK), repeated frame (ACK again, no data),
-- waiting for credit with STATUS, START reset.  The raw responses are
-- written to G_RESP for a check with the Python parser.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

entity tb_link is
    generic (
        G_IN     : string := "in.264";          -- original stream
        G_FRAMES : string := "frames.txt";      -- make_vectors.py output
        G_RESP   : string := "responses.bin";
        G_DIV    : positive := 8;               -- clock cycles per UART bit
        G_AW     : positive := 11               -- FIFO 2 KiB: forces credit waits
    );
end entity;

architecture sim of tb_link is
    constant TCLK : time := 10 ns;
    constant TBIT : time := G_DIV * TCLK;

    signal clk     : std_logic := '0';
    signal rst     : std_logic := '1';
    signal h2f     : std_logic := '1';      -- host -> FPGA line
    signal f2h     : std_logic;             -- FPGA -> host line
    signal rx_data, tx_data : std_logic_vector(7 downto 0);
    signal rx_valid, tx_valid, tx_ready : std_logic;
    signal tdata   : std_logic_vector(7 downto 0);
    signal tvalid, tlast, tready : std_logic;
    signal dec_rst : std_logic;
    signal done_s  : std_logic := '0';
    signal lfsr    : std_logic_vector(15 downto 0) := x"ACE1";
    signal stall   : boolean := true;

    -- responses decoded by the host receiver
    signal r_evt   : natural := 0;
    signal r_type, r_seq, r_flags : natural;
    signal r_free, r_w, r_h : natural;
    signal r_pics  : natural;
    signal sink_n  : natural := 0;          -- bytes received by the sink
    signal sink_end : boolean := false;
    signal finished : boolean := false;

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

    u_rx : entity work.h264_uart_rx
        generic map (G_DIV => G_DIV)
        port map (clk => clk, rst => rst, i_rx => h2f, o_data => rx_data, o_valid => rx_valid, o_ferr => open);
    u_tx : entity work.h264_uart_tx
        generic map (G_DIV => G_DIV)
        port map (clk => clk, rst => rst, i_data => tx_data, i_valid => tx_valid, o_ready => tx_ready, o_tx => f2h);
    dut : entity work.h264_link
        generic map (G_FIFO_AW => G_AW)
        port map (clk => clk, rst => rst, i_rx_data => rx_data, i_rx_valid => rx_valid,
                  o_tx_data => tx_data, o_tx_valid => tx_valid, i_tx_ready => tx_ready,
                  m_axis_tdata => tdata, m_axis_tvalid => tvalid, m_axis_tlast => tlast,
                  m_axis_tready => tready, o_dec_rst => dec_rst,
                  i_pic_count => x"0000002A", i_error => '0', i_stream_done => done_s,
                  i_width => x"0B0", i_height => x"090", o_frame_ok => open, o_frame_bad => open);

    process (clk)
    begin
        if rising_edge(clk) then
            lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
        end if;
    end process;
    tready <= '0' when stall else lfsr(0) or lfsr(5);

    -- ------------------------------------------------------------------
    -- sink: compare with the original file
    -- ------------------------------------------------------------------
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

    -- ------------------------------------------------------------------
    -- host receiver: 16 byte responses
    -- ------------------------------------------------------------------
    host_rx : process
        file fo : char_file_t;
        variable b : natural;
        variable buf : integer_vector(0 to 15);
        variable cnt : natural := 0;
        variable crc : natural;
        variable st : file_open_status;
    begin
        file_open(st, fo, G_RESP, write_mode);
        loop
            wait until f2h = '0';
            wait for TBIT / 2;
            assert f2h = '0' report "start bit glitch" severity error;
            b := 0;
            for i in 0 to 7 loop
                wait for TBIT;
                if f2h = '1' then
                    b := b + 2 ** i;
                end if;
            end loop;
            wait for TBIT;
            assert f2h = '1' report "stop bit missing" severity error;
            write(fo, character'val(b));
            -- resynchronise on 5A A5
            if cnt = 0 and b /= 16#5A# then
                next;
            elsif cnt = 1 and b /= 16#A5# then
                cnt := 0;
                if b = 16#5A# then
                    buf(0) := b;
                    cnt := 1;
                end if;
                next;
            end if;
            buf(cnt) := b;
            cnt := cnt + 1;
            if cnt = 16 then
                cnt := 0;
                crc := 16#FFFF#;
                for i in 2 to 13 loop
                    crc := crc16_upd(crc, buf(i));
                end loop;
                assert crc = buf(14) + 256 * buf(15) report "response CRC wrong" severity failure;
                r_type <= buf(2);
                r_seq <= buf(3);
                r_free <= buf(4) + 256 * buf(5);
                r_pics <= buf(6) + 256 * buf(7);
                r_flags <= buf(10);
                r_w <= buf(11) + 256 * (buf(12) mod 16);
                r_h <= buf(12) / 16 + 16 * buf(13);
                r_evt <= r_evt + 1;
            end if;
        end loop;
    end process;

    -- ------------------------------------------------------------------
    -- host: send the frames with the directed error cases
    -- ------------------------------------------------------------------
    host : process
        file ff : text;
        variable l : line;
        type bytes_t is array (0 to 2097151) of natural range 0 to 255;
        variable fb : bytes_t;
        type idx_t is array (0 to 8191) of natural;
        variable fo, fl : idx_t;           -- offset / length of frame k
        type kind_t is array (0 to 8191) of character;
        variable kind : kind_t;
        variable nf, pos : natural := 0;
        variable ch : character;
        variable v : natural;
        variable good : boolean;
        variable free, seq, plen, ndata, j : natural;
        variable polls, credit_waits : natural := 0;
        variable st : file_open_status;

        function hexval(c : character) return natural is
        begin
            if c >= '0' and c <= '9' then
                return character'pos(c) - character'pos('0');
            end if;
            return character'pos(c) - character'pos('a') + 10;
        end function;

        procedure send_byte(b : natural) is
            variable u : unsigned(7 downto 0);
        begin
            u := to_unsigned(b, 8);
            h2f <= '0';
            wait for TBIT;
            for i in 0 to 7 loop
                h2f <= u(i);
                wait for TBIT;
            end loop;
            h2f <= '1';
            wait for TBIT;
        end procedure;

        -- send frame k; corrupt = byte index to flip (or -1)
        procedure send_frame(k : natural; corrupt : integer) is
            variable b : natural;
        begin
            for i in 0 to fl(k) - 1 loop
                b := fb(fo(k) + i);
                if i = corrupt then
                    b := (b + 16) mod 256;     -- flip bit 4
                end if;
                send_byte(b);
            end loop;
        end procedure;

        procedure expect(t, s : natural; what : string) is
        begin
            wait on r_evt for 5 ms;
            assert r_evt'event report "no response (" & what & ")" severity failure;
            assert r_type = t and r_seq = s
                report what & ": got type " & integer'image(r_type) & " seq " & integer'image(r_seq) &
                       ", expected " & integer'image(t) & " / " & integer'image(s) severity failure;
            free := r_free;
        end procedure;

        -- frame index of the status request
        variable kq, kd0, kend : natural;
    begin
        file_open(st, ff, G_FRAMES, read_mode);
        assert st = open_ok report "cannot open " & G_FRAMES severity failure;
        while not endfile(ff) loop
            readline(ff, l);
            exit when l'length < 3;
            read(l, ch);
            kind(nf) := ch;
            fo(nf) := pos;
            while l'length > 1 loop
                read(l, ch);
                v := hexval(ch) * 16;
                read(l, ch);
                v := v + hexval(ch);
                fb(pos) := v;
                pos := pos + 1;
            end loop;
            fl(nf) := pos - fo(nf);
            nf := nf + 1;
        end loop;
        kq := 1;
        kd0 := 2;
        kend := nf - 1;
        assert kind(0) = 'S' and kind(kq) = 'Q' and kind(kend) = 'E' report "unexpected frame file" severity failure;

        wait for 200 ns;
        rst <= '0';
        wait for 1 us;

        -- reset and status
        send_frame(0, -1);
        expect(16#81#, 0, "START");
        assert free = 2 ** G_AW - 1 report "free after START" severity failure;
        send_frame(kq, -1);
        expect(16#83#, 0, "STATUS");
        assert r_pics = 42 and r_w = 176 and r_h = 144 report "status fields" severity failure;

        ndata := 0;
        k_loop : for k in kd0 to kend - 1 loop
            seq := (k - 1) mod 256;
            plen := fb(fo(k) + 4) + 256 * fb(fo(k) + 5);
            j := k - kd0;
            -- wait for credit (the sink starts draining after a few polls)
            polls := 0;
            while plen > free loop
                wait for 20 us;
                send_frame(kq, -1);
                expect(16#83#, (seq - 1) mod 256, "STATUS while waiting for credit");
                polls := polls + 1;
                if polls = 5 then
                    stall <= false;
                end if;
                credit_waits := credit_waits + 1;
            end loop;
            if j mod 5 = 3 then
                send_frame(k, 6 + plen / 2);       -- corrupted payload byte
                expect(16#82#, seq, "NAK after CRC error");
                send_frame(k, -1);
                expect(16#81#, seq, "ACK after resend");
            elsif j mod 7 = 4 and kind(k + 1) = 'D' then
                send_frame(k + 1, -1);             -- next frame first
                expect(16#82#, seq, "NAK for a frame out of order");
                send_frame(k, -1);
                expect(16#81#, seq, "ACK in order");
            else
                send_frame(k, -1);
                expect(16#81#, seq, "ACK");
            end if;
            if j mod 6 = 2 then
                send_frame(k, -1);                 -- repeated frame: acknowledged, not stored
                expect(16#81#, seq, "ACK for a repeated frame");
            end if;
            ndata := ndata + 1;
        end loop;
        stall <= false;
        send_frame(kend, -1);
        expect(16#81#, (kend - 1) mod 256, "END");

        wait until sink_end for 10 ms;
        assert sink_end report "stream end (TLAST) not received" severity failure;
        wait for 1 us;
        send_frame(kq, -1);
        expect(16#83#, (kend - 1) mod 256, "STATUS at the end");
        assert r_flags = 16#0E# report "flags at the end: " & integer'image(r_flags) severity failure;
        assert free = 2 ** G_AW - 1 report "FIFO not empty at the end" severity failure;

        -- START again: decoder reset pulse, link reset
        send_frame(0, -1);
        wait until dec_rst = '1' for 2 ms;
        assert dec_rst = '1' report "no decoder reset after START" severity failure;
        expect(16#81#, 0, "second START");

        assert credit_waits > 0 report "credit wait not exercised" severity failure;
        report "link test passed: " & integer'image(ndata) & " data frames, " &
               integer'image(sink_n) & " bytes, last byte with TLAST, " &
               integer'image(credit_waits) & " credit polls";
        finished <= true;
        wait;
    end process;
end architecture;
