-- ============================================================================
-- h264_tmds.vhd
-- DVI 1.0 TMDS encoder for one channel, four symbols per clock (pixel j of
-- the group in bits 10*j+9 .. 10*j of the output word, bit 0 of a symbol is
-- the first one on the wire).  The running disparity is carried from symbol
-- to symbol inside the group and across clocks.  Latency 3 clocks.
--   de = 1: data symbol of d (8 bit); de = 0: control symbol for (c1, c0)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_tmds4 is
    port (
        clk  : in  std_logic;
        rst  : in  std_logic;
        i_de : in  std_logic_vector(3 downto 0);
        i_c0 : in  std_logic_vector(3 downto 0);
        i_c1 : in  std_logic_vector(3 downto 0);
        i_d  : in  std_logic_vector(31 downto 0);
        o_q  : out std_logic_vector(39 downto 0)
    );
end entity;

architecture rtl of h264_tmds4 is
    type qm_t is array (0 to 3) of std_logic_vector(8 downto 0);
    type n_t is array (0 to 3) of unsigned(3 downto 0);
    signal qm0, qm : qm_t;
    signal n1  : n_t;                       -- ones in qm(7..0)
    signal de0, c00, c10 : std_logic_vector(3 downto 0);
    signal de1, c01, c11 : std_logic_vector(3 downto 0);
    signal cnt : signed(4 downto 0);        -- running disparity (even, |cnt| <= 10)

    function ones8(v : std_logic_vector(7 downto 0)) return unsigned is
        variable n : unsigned(3 downto 0) := (others => '0');
    begin
        for i in 0 to 7 loop
            if v(i) = '1' then
                n := n + 1;
            end if;
        end loop;
        return n;
    end function;
begin
    -- stage 1: transition minimised word (independent of the disparity)
    process (clk)
        variable d : std_logic_vector(7 downto 0);
        variable q : std_logic_vector(8 downto 0);
        variable n : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            for j in 0 to 3 loop
                d := i_d(8 * j + 7 downto 8 * j);
                n := ones8(d);
                q(0) := d(0);
                if n > 4 or (n = 4 and d(0) = '0') then
                    for i in 1 to 7 loop
                        q(i) := q(i - 1) xnor d(i);
                    end loop;
                    q(8) := '0';
                else
                    for i in 1 to 7 loop
                        q(i) := q(i - 1) xor d(i);
                    end loop;
                    q(8) := '1';
                end if;
                qm0(j) <= q;
            end loop;
            de0 <= i_de;
            c00 <= i_c0;
            c10 <= i_c1;
            -- stage 1b: ones of the transition minimised word
            for j in 0 to 3 loop
                qm(j) <= qm0(j);
                n1(j) <= ones8(qm0(j)(7 downto 0));
            end loop;
            de1 <= de0;
            c01 <= c00;
            c11 <= c10;
        end if;
    end process;

    -- stage 2: DC balancing, chained over the four symbols
    process (clk)
        variable c : signed(4 downto 0);
        variable diff : signed(4 downto 0);    -- n1 - n0 of qm(7..0)
        variable q : std_logic_vector(9 downto 0);
        variable sel : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            c := cnt;
            for j in 0 to 3 loop
                diff := signed(resize(n1(j), 5)) - signed(resize(8 - n1(j), 5));
                if de1(j) = '0' then
                    sel := c11(j) & c01(j);
                    case sel is
                        when "00" => q := "1101010100";
                        when "01" => q := "0010101011";
                        when "10" => q := "0101010100";
                        when others => q := "1010101011";
                    end case;
                    c := (others => '0');
                elsif c = 0 or diff = 0 then
                    q(9) := not qm(j)(8);
                    q(8) := qm(j)(8);
                    if qm(j)(8) = '1' then
                        q(7 downto 0) := qm(j)(7 downto 0);
                        c := c + diff;
                    else
                        q(7 downto 0) := not qm(j)(7 downto 0);
                        c := c - diff;
                    end if;
                elsif (c > 0 and diff > 0) or (c < 0 and diff < 0) then
                    q := '1' & qm(j)(8) & not qm(j)(7 downto 0);
                    c := c - diff;
                    if qm(j)(8) = '1' then
                        c := c + 2;
                    end if;
                else
                    q := '0' & qm(j)(8) & qm(j)(7 downto 0);
                    c := c + diff;
                    if qm(j)(8) = '0' then
                        c := c - 2;
                    end if;
                end if;
                o_q(10 * j + 9 downto 10 * j) <= q;
            end loop;
            cnt <= c;
            if rst = '1' then
                cnt <= (others => '0');
            end if;
        end if;
    end process;
end architecture;
