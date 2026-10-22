-- ============================================================================
-- tb_parse.vhd
-- Front-end testbench: byte stream -> h264_nal -> h264_bitreader ->
-- h264_hdr + h264_mbparse.  Every macroblock token and its coefficient blocks
-- are written in the format of the C model parser trace (h264_model -p).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.h264_pkg.all;
use work.tb_util_pkg.all;

entity tb_parse is
    generic (
        G_IN    : string := "in.264";
        G_TRACE : string := "rtl_trace.txt";
        G_TIMEOUT_US : natural := 2000000
    );
end entity;

architecture sim of tb_parse is
    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal s_tdata  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_tvalid : std_logic := '0';
    signal s_tlast  : std_logic := '0';
    signal s_tready : std_logic;

    signal n_data   : std_logic_vector(7 downto 0);
    signal n_first, n_last, n_eos, n_valid, n_ready : std_logic;

    signal br_win    : std_logic_vector(63 downto 0);
    signal br_nbits  : unsigned(6 downto 0);
    signal br_avail32, br_avail64, br_more, br_active, br_eos : std_logic;
    signal br_bitpos : unsigned(31 downto 0);
    signal br_consume : unsigned(6 downto 0);
    signal br_flush  : std_logic;

    signal hdr_consume, mbp_consume : unsigned(6 downto 0);
    signal hdr_own : std_logic;
    signal slice : slice_t;
    signal slice_start, slice_done : std_logic;
    signal pic_done : std_logic := '0';
    signal pic_done_fb : unsigned(4 downto 0) := (others => '0');
    signal frame_valid : std_logic;
    signal frame_fb : unsigned(4 downto 0);
    signal rel_pulse : std_logic := '0';
    signal rel_fb : unsigned(4 downto 0) := (others => '0');
    signal hdr_err, mbp_err, stream_done : std_logic;

    signal cw_en : std_logic;
    signal cw_addr : unsigned(8 downto 0);
    signal cw_data : std_logic_vector(63 downto 0);
    signal coef_free : std_logic := '0';
    signal mb : mbinfo_t;
    signal mb_valid : std_logic;

    type coef_ram_t is array (0 to 511) of std_logic_vector(63 downto 0);
    signal coef_ram : coef_ram_t;
    signal done : boolean := false;
begin
    clk <= not clk after 5 ns when not done;

    u_nal : entity work.h264_nal
        port map (clk => clk, rst => rst, s_tdata => s_tdata, s_tvalid => s_tvalid, s_tlast => s_tlast,
                  s_tready => s_tready, o_data => n_data, o_first => n_first, o_last => n_last,
                  o_eos => n_eos, o_valid => n_valid, i_ready => n_ready);

    br_consume <= hdr_consume when hdr_own = '1' else mbp_consume;

    u_br : entity work.h264_bitreader
        port map (clk => clk, rst => rst, i_data => n_data, i_first => n_first, i_last => n_last,
                  i_eos => n_eos, i_valid => n_valid, o_ready => n_ready, o_win => br_win,
                  o_nbits => br_nbits, o_avail32 => br_avail32, o_avail64 => br_avail64,
                  o_more => br_more, o_active => br_active, o_eos => br_eos, o_bitpos => br_bitpos,
                  i_consume => br_consume, i_flush => br_flush);

    u_hdr : entity work.h264_hdr
        port map (clk => clk, rst => rst, br_win => br_win, br_avail64 => br_avail64, br_more => br_more,
                  br_active => br_active, br_eos => br_eos, hdr_consume => hdr_consume, br_flush => br_flush,
                  own_br => hdr_own, slice => slice, slice_start => slice_start, slice_done => slice_done,
                  pic_done => pic_done, pic_done_fb => pic_done_fb, o_frame_valid => frame_valid,
                  o_frame_fb => frame_fb, i_frame_ready => '1', i_release => rel_pulse,
                  i_release_fb => rel_fb, o_mb_w => open, o_mb_h => open, o_crop_l => open,
                  o_crop_r => open, o_crop_t => open, o_crop_b => open, o_error => hdr_err,
                  o_stream_done => stream_done, o_pic_count => open);

    u_mbp : entity work.h264_mbparse
        port map (clk => clk, rst => rst, br_win => br_win, br_avail32 => br_avail32, br_more => br_more,
                  br_bitpos => br_bitpos, mbp_consume => mbp_consume, slice => slice,
                  slice_start => slice_start, slice_done => slice_done, o_error => mbp_err,
                  cw_en => cw_en, cw_addr => cw_addr, cw_data => cw_data, i_coef_free => coef_free,
                  o_mb => mb, o_mb_valid => mb_valid, i_mb_ready => '1');

    -- stream feeder
    feed : process
        file f : char_file_t;
        variable c : character;
        variable st : file_open_status;
    begin
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

    -- coefficient buffer model
    process (clk)
    begin
        if rising_edge(clk) then
            if cw_en = '1' then
                coef_ram(to_integer(cw_addr)) <= cw_data;
            end if;
        end if;
    end process;

    -- frames are released immediately
    process (clk)
    begin
        if rising_edge(clk) then
            rel_pulse <= frame_valid;
            rel_fb <= frame_fb;
        end if;
    end process;

    -- token monitor
    mon : process (clk)
        file tf : text open write_mode is G_TRACE;
        variable l : line;
        variable a : integer;
        variable word : std_logic_vector(63 downto 0);
        variable v : integer;
        variable fl, ft : boolean;
        variable bsc : integer;
        variable nmb : integer := 0;
    begin
        if rising_edge(clk) then
            coef_free <= '0';
            pic_done <= '0';
            if mb_valid = '1' then
                coef_free <= '1';
                nmb := nmb + 1;
                if mb.last_of_pic = '1' then
                    pic_done <= '1';
                    pic_done_fb <= mb.cur_fb;
                end if;
                a := to_integer(mb.mb_y) * to_integer(mb.pic_w) + to_integer(mb.mb_x);
                wr(l, "T " & itoa(a) & " t=" & itoa(to_integer(mb.mbtype)) & " qp=" & itoa(to_integer(mb.qp)) &
                      " qpc=" & itoa(to_integer(mb.qpc)));
                if mb.mbtype = MBT_I4x4 then
                    wr(l, " m=");
                    for i in 0 to 15 loop
                        wr(l, hex(std_logic_vector(mb.i4_modes(i)), 1));
                    end loop;
                end if;
                if mb.mbtype = MBT_I16 then
                    wr(l, " m=" & itoa(to_integer(mb.i16_mode)));
                end if;
                if mb.mbtype = MBT_I4x4 or mb.mbtype = MBT_I16 then
                    wr(l, " c=" & itoa(to_integer(mb.chroma_mode)) & " av=" & std_logic'image(mb.avail_a)(2) &
                          std_logic'image(mb.avail_b)(2) & std_logic'image(mb.avail_c)(2) &
                          std_logic'image(mb.avail_d)(2));
                end if;
                if not mbt_is_intra(mb.mbtype) then
                    wr(l, " p=" & itoa(to_integer(mb.part)) & " s=" & itoa(to_integer(mb.sub(0))) &
                          itoa(to_integer(mb.sub(1))) & itoa(to_integer(mb.sub(2))) & itoa(to_integer(mb.sub(3))) & " mv=");
                    for i in 0 to 15 loop
                        wr(l, itoa(to_integer(mb.mv(i).x)) & "," & itoa(to_integer(mb.mv(i).y)) & ";");
                    end loop;
                    wr(l, " rp=" & itoa(to_integer(mb.refpic(0))) & "," & itoa(to_integer(mb.refpic(1))) & "," &
                          itoa(to_integer(mb.refpic(2))) & "," & itoa(to_integer(mb.refpic(3))));
                end if;
                wr(l, " nz=" & hex(mb.nz, 7));
                fl := mb.filt_left = '1';
                ft := mb.filt_top = '1';
                wr(l, " f=" & std_logic'image(mb.filt_left)(2) & std_logic'image(mb.filt_top)(2) &
                      std_logic'image(mb.filt_int)(2) & " qdb=" & itoa(to_integer(mb.qp_db)));
                if fl then
                    wr(l, " ql=" & itoa(to_integer(mb.qp_left)));
                end if;
                if ft then
                    wr(l, " qt=" & itoa(to_integer(mb.qp_top)));
                end if;
                if mb.filt_int = '1' then
                    wr(l, " bs=");
                    for i in 0 to 31 loop
                        if (i mod 16) / 4 = 0 and ((i < 16 and not fl) or (i >= 16 and not ft)) then
                            wr(l, "x");
                        else
                            wr(l, itoa(to_integer(mb.bs(i))));
                        end if;
                    end loop;
                end if;
                writeline(tf, l);
                for b in 0 to 25 loop
                    if mb.nz(b) = '1' then
                        wr(l, "C " & itoa(a) & " " & itoa(b));
                        for i in 0 to 15 loop
                            if b = 17 then
                                if i < 8 then
                                    word := coef_ram(to_integer(mb.coef_slot) * 128 + 17 * 4 + i / 4);
                                    v := to_integer(signed(word((i mod 4) * 16 + 15 downto (i mod 4) * 16)));
                                else
                                    v := 0;
                                end if;
                            else
                                word := coef_ram(to_integer(mb.coef_slot) * 128 + b * 4 + i / 4);
                                v := to_integer(signed(word((i mod 4) * 16 + 15 downto (i mod 4) * 16)));
                                if mb.mbtype = MBT_IPCM then
                                    v := v mod 256;
                                end if;
                            end if;
                            wr(l, " " & itoa(v));
                        end loop;
                        writeline(tf, l);
                    end if;
                end loop;
            end if;
            if stream_done = '1' and not done then
                report "stream done, " & integer'image(nmb) & " macroblocks, errors hdr=" &
                       std_logic'image(hdr_err) & " mbp=" & std_logic'image(mbp_err);
                done <= true;
            end if;
        end if;
    end process;

    -- watchdog
    process
    begin
        wait for G_TIMEOUT_US * 1 us;
        report "timeout" severity failure;
        wait;
    end process;
end architecture;
