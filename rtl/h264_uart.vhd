-- ============================================================================
-- h264_uart.vhd
-- 8N1 UART receiver and transmitter in the fabric (no MSS).
--   G_DIV = clock cycles per bit (e.g. 100 MHz / 2 Mbaud = 50).  The
--   receiver samples in the middle of each bit (start bit checked again at
--   half a bit); a byte with a missing stop bit is dropped.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_uart_rx is
    generic (
        G_DIV : positive := 50
    );
    port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        i_rx    : in  std_logic;
        o_data  : out std_logic_vector(7 downto 0);
        o_valid : out std_logic;                 -- one cycle per received byte
        o_ferr  : out std_logic                  -- one cycle per framing error
    );
end entity;

architecture rtl of h264_uart_rx is
    signal rx_s  : std_logic_vector(2 downto 0) := (others => '1');   -- synchroniser
    signal busy  : std_logic;
    signal cnt   : natural range 0 to G_DIV - 1;
    signal bitn  : natural range 0 to 9;
    signal sh    : std_logic_vector(7 downto 0);
begin
    process (clk)
        variable rxd : std_logic;
    begin
        if rising_edge(clk) then
            rx_s <= rx_s(1 downto 0) & i_rx;
            rxd := rx_s(2);
            o_valid <= '0';
            o_ferr <= '0';
            if rst = '1' then
                busy <= '0';
                cnt <= 0;
                bitn <= 0;
            elsif busy = '0' then
                if rxd = '0' then
                    -- start bit edge: check it again at half a bit
                    busy <= '1';
                    cnt <= G_DIV / 2;
                    bitn <= 0;
                end if;
            elsif cnt /= G_DIV - 1 then
                cnt <= cnt + 1;
            else
                cnt <= 0;
                if bitn = 0 then
                    if rxd = '1' then
                        busy <= '0';            -- glitch, not a start bit
                    end if;
                    bitn <= 1;
                elsif bitn <= 8 then
                    sh <= rxd & sh(7 downto 1);  -- LSB first
                    bitn <= bitn + 1;
                else
                    busy <= '0';
                    if rxd = '1' then
                        o_data <= sh;
                        o_valid <= '1';
                    else
                        o_ferr <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process;
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_uart_tx is
    generic (
        G_DIV : positive := 50
    );
    port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        i_data  : in  std_logic_vector(7 downto 0);
        i_valid : in  std_logic;
        o_ready : out std_logic;
        o_tx    : out std_logic
    );
end entity;

architecture rtl of h264_uart_tx is
    signal busy : std_logic;
    signal cnt  : natural range 0 to G_DIV - 1;
    signal bitn : natural range 0 to 9;
    signal sh   : std_logic_vector(8 downto 0);    -- data & stop, start sent first
    signal txr  : std_logic;
begin
    o_ready <= not busy;
    o_tx <= txr;

    process (clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                busy <= '0';
                txr <= '1';
                cnt <= 0;
                bitn <= 0;
            elsif busy = '0' then
                txr <= '1';
                if i_valid = '1' then
                    busy <= '1';
                    txr <= '0';                 -- start bit
                    sh <= '1' & i_data;
                    cnt <= 0;
                    bitn <= 0;
                end if;
            elsif cnt /= G_DIV - 1 then
                cnt <= cnt + 1;
            else
                cnt <= 0;
                if bitn = 9 then
                    busy <= '0';                -- stop bit done
                    txr <= '1';
                else
                    txr <= sh(0);
                    sh <= '1' & sh(8 downto 1);
                    bitn <= bitn + 1;
                end if;
            end if;
        end if;
    end process;
end architecture;
