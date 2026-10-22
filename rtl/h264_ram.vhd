-- ============================================================================
-- h264_ram.vhd
-- Generic memories used between the pipeline stages.
--   h264_ram_sdp : simple dual port RAM, synchronous read (LSRAM / uSRAM)
--   h264_pixbuf  : 16 byte-wide banks sharing one address; a write enables
--                  any subset of banks, a read returns all 16 bytes.
--                  Used for macroblock pixel buffers: address = slot*32+row,
--                  luma rows 0..15 (banks 0..15 = x), chroma rows 16..23
--                  (banks 0..7 = Cb x, banks 8..15 = Cr x).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_ram_sdp is
    generic (
        G_AW : natural := 9;
        G_DW : natural := 64
    );
    port (
        clk   : in  std_logic;
        we    : in  std_logic;
        waddr : in  unsigned(G_AW - 1 downto 0);
        wdata : in  std_logic_vector(G_DW - 1 downto 0);
        raddr : in  unsigned(G_AW - 1 downto 0);
        rdata : out std_logic_vector(G_DW - 1 downto 0)
    );
end entity;

architecture rtl of h264_ram_sdp is
    type mem_t is array (0 to 2 ** G_AW - 1) of std_logic_vector(G_DW - 1 downto 0);
    signal mem : mem_t := (others => (others => '0'));
begin
    process (clk)
    begin
        if rising_edge(clk) then
            if we = '1' then
                mem(to_integer(waddr)) <= wdata;
            end if;
            rdata <= mem(to_integer(raddr));
        end if;
    end process;
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_pixbuf is
    generic (
        G_AW : natural := 6
    );
    port (
        clk   : in  std_logic;
        we    : in  std_logic_vector(15 downto 0);
        waddr : in  unsigned(G_AW - 1 downto 0);
        wdata : in  std_logic_vector(127 downto 0);
        raddr : in  unsigned(G_AW - 1 downto 0);
        rdata : out std_logic_vector(127 downto 0)
    );
end entity;

architecture rtl of h264_pixbuf is
begin
    g_bank : for b in 0 to 15 generate
        u_bank : entity work.h264_ram_sdp
            generic map (G_AW => G_AW, G_DW => 8)
            port map (clk => clk, we => we(b), waddr => waddr, wdata => wdata(b * 8 + 7 downto b * 8),
                      raddr => raddr, rdata => rdata(b * 8 + 7 downto b * 8));
    end generate;
end architecture;
