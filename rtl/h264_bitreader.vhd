-- ============================================================================
-- h264_bitreader.vhd
-- RBSP bit window over the NAL byte stream.
--   o_win(63) is the next bit.  The window is the top of a 72 bit buffer so
--   that at least 64 valid bits can always be reached.  Bits beyond the end
--   of the NAL read as zero.  Bytes are only loaded up to the last byte of the
--   current NAL; i_flush drops the rest of the NAL and arms the reader for
--   the next one.
--   o_avail32/o_avail64: at least 32/64 bits valid or the NAL end is loaded.
--   o_more: more_rbsp_data() (valid when o_avail32 = '1').
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_bitreader is
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        -- byte input from h264_nal
        i_data     : in  std_logic_vector(7 downto 0);
        i_first    : in  std_logic;
        i_last     : in  std_logic;
        i_eos      : in  std_logic;
        i_valid    : in  std_logic;
        o_ready    : out std_logic;
        -- window
        o_win      : out std_logic_vector(63 downto 0);
        o_nbits    : out unsigned(6 downto 0);   -- valid bits in the buffer (0..72)
        o_avail32  : out std_logic;
        o_avail64  : out std_logic;
        o_more     : out std_logic;
        o_active   : out std_logic;     -- a NAL is loaded
        o_eos      : out std_logic;     -- the last byte of the stream was seen (sticky)
        o_bitpos   : out unsigned(31 downto 0);  -- bits consumed in this NAL
        -- control
        i_consume  : in  unsigned(6 downto 0);   -- bits to drop (0..64)
        i_flush    : in  std_logic
    );
end entity;

architecture rtl of h264_bitreader is
    signal buf      : std_logic_vector(71 downto 0);
    signal nbits    : unsigned(6 downto 0);
    signal active   : std_logic;
    signal lastin   : std_logic;
    signal eos_r    : std_logic;
    signal tail     : unsigned(3 downto 0);
    signal bitpos   : unsigned(31 downto 0);
    signal rdy      : std_logic;
begin
    o_win <= buf(71 downto 8);
    o_nbits <= nbits;
    o_avail32 <= '1' when nbits >= 32 or lastin = '1' else '0';
    o_avail64 <= '1' when nbits >= 64 or lastin = '1' else '0';
    o_more <= '1' when lastin = '0' or nbits > resize(tail, 7) else '0';
    o_active <= active;
    o_eos <= eos_r;
    o_bitpos <= bitpos;

    -- accept a byte when it can be placed in the window this cycle
    process (all)
        variable n1 : integer;
    begin
        n1 := to_integer(nbits) - to_integer(i_consume);
        if n1 < 0 then
            n1 := 0;
        end if;
        rdy <= '0';
        if i_flush = '0' then
            if active = '0' then
                -- waiting for a NAL start; drop bytes that do not start one
                rdy <= '1';
            elsif lastin = '0' and n1 <= 64 then
                rdy <= '1';
            end if;
        end if;
    end process;
    o_ready <= rdy;

    process (clk)
        variable n1  : integer range 0 to 72;
        variable b1  : std_logic_vector(71 downto 0);
        variable sh  : std_logic_vector(71 downto 0);
        variable tz  : natural range 0 to 7;
        variable c   : integer;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                buf <= (others => '0');
                nbits <= (others => '0');
                active <= '0';
                lastin <= '0';
                eos_r <= '0';
                tail <= (others => '0');
                bitpos <= (others => '0');
            elsif i_flush = '1' then
                buf <= (others => '0');
                nbits <= (others => '0');
                active <= '0';
                lastin <= '0';
                bitpos <= (others => '0');
            else
                c := to_integer(i_consume);
                if c > to_integer(nbits) then
                    n1 := 0;
                else
                    n1 := to_integer(nbits) - c;
                end if;
                b1 := std_logic_vector(shift_left(unsigned(buf), c));
                bitpos <= bitpos + c;
                if i_valid = '1' and rdy = '1' then
                    if i_eos = '1' then
                        eos_r <= '1';
                    end if;
                    if active = '0' then
                        if i_first = '1' then
                            active <= '1';
                            b1(71 downto 64) := i_data;
                            n1 := 8;
                            bitpos <= (others => '0');
                            if i_last = '1' then
                                lastin <= '1';
                            end if;
                        end if;
                    else
                        sh := (others => '0');
                        sh(71 downto 64) := i_data;
                        sh := std_logic_vector(shift_right(unsigned(sh), n1));
                        b1 := b1 or sh;
                        n1 := n1 + 8;
                        if i_last = '1' then
                            lastin <= '1';
                        end if;
                    end if;
                    if i_last = '1' then
                        tz := 0;
                        for k in 0 to 7 loop
                            if i_data(k) = '1' then
                                tz := k;
                                exit;
                            end if;
                        end loop;
                        tail <= to_unsigned(tz + 1, 4);
                    end if;
                end if;
                buf <= b1;
                nbits <= to_unsigned(n1, 7);
            end if;
        end if;
    end process;
end architecture;
