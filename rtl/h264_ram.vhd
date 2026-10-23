-- ============================================================================
-- h264_ram.vhd
-- Generic memories used between the pipeline stages.
--   h264_ram_sdp  : simple dual port RAM, synchronous read (LSRAM / uSRAM)
--   h264_ram_dc   : the same with separate write and read clocks (LSRAM)
--   h264_coefbuf  : coefficient buffer (4 row lanes, one 4x4 block per address)
--   h264_predbuf  : inter prediction buffer (registers, row write, block read)
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
    signal mem : mem_t;
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

-- ----------------------------------------------------------------------------
-- h264_ram_dc: simple dual port RAM, write port on wclk, registered read port
-- on rclk (display line buffers between the AXI and the pixel clock).
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_ram_dc is
    generic (
        G_AW : natural := 9;
        G_DW : natural := 64
    );
    port (
        wclk  : in  std_logic;
        we    : in  std_logic;
        waddr : in  unsigned(G_AW - 1 downto 0);
        wdata : in  std_logic_vector(G_DW - 1 downto 0);
        rclk  : in  std_logic;
        raddr : in  unsigned(G_AW - 1 downto 0);
        rdata : out std_logic_vector(G_DW - 1 downto 0)
    );
end entity;

architecture rtl of h264_ram_dc is
    type mem_t is array (0 to 2 ** G_AW - 1) of std_logic_vector(G_DW - 1 downto 0);
    signal mem : mem_t;
begin
    process (wclk)
    begin
        if rising_edge(wclk) then
            if we = '1' then
                mem(to_integer(waddr)) <= wdata;
            end if;
        end if;
    end process;

    process (rclk)
    begin
        if rising_edge(rclk) then
            rdata <= mem(to_integer(raddr));
        end if;
    end process;
end architecture;

-- ----------------------------------------------------------------------------
-- h264_coefbuf: coefficient buffer, one 4x4 block of 16 bit values per
-- address (slot & block); written per row lane (we(r): row r at bits
-- r*64+63 .. r*64), read as a whole block.
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_coefbuf is
    port (
        clk   : in  std_logic;
        we    : in  std_logic_vector(3 downto 0);
        waddr : in  unsigned(6 downto 0);
        wdata : in  std_logic_vector(255 downto 0);
        raddr : in  unsigned(6 downto 0);
        rdata : out std_logic_vector(255 downto 0)
    );
end entity;

architecture rtl of h264_coefbuf is
begin
    g_lane : for r in 0 to 3 generate
        u_lane : entity work.h264_ram_sdp
            generic map (G_AW => 7, G_DW => 64)
            port map (clk => clk, we => we(r), waddr => waddr, wdata => wdata(r * 64 + 63 downto r * 64),
                      raddr => raddr, rdata => rdata(r * 64 + 63 downto r * 64));
    end generate;
end architecture;

-- ----------------------------------------------------------------------------
-- h264_predbuf: inter prediction buffer (2 macroblock slots, registers).
--   write: one row of up to 16 samples with per-sample enables,
--          address = slot & row (luma rows 0..15, chroma rows 16..23:
--          samples 0..7 Cb, 8..15 Cr)
--   read : one 4x4 block (registered output), block 0..15 luma raster,
--          16..19 Cb, 20..23 Cr; sample i at bits i*8+7 .. i*8
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_predbuf is
    port (
        clk   : in  std_logic;
        we    : in  std_logic_vector(15 downto 0);
        waddr : in  unsigned(5 downto 0);
        wdata : in  std_logic_vector(127 downto 0);
        rslot : in  std_logic;
        rblk  : in  unsigned(4 downto 0);
        rdata : out std_logic_vector(127 downto 0)
    );
end entity;

architecture rtl of h264_predbuf is
    type row_t is array (0 to 15) of std_logic_vector(7 downto 0);
    type buf_t is array (0 to 47) of row_t;       -- slot*24 + row
    signal mem : buf_t;
begin
    process (clk)
        variable a : natural;
    begin
        if rising_edge(clk) then
            a := to_integer(waddr(5 downto 5)) * 24 + to_integer(waddr(4 downto 0));
            if a < 48 then
                for i in 0 to 15 loop
                    if we(i) = '1' then
                        mem(a)(i) <= wdata(i * 8 + 7 downto i * 8);
                    end if;
                end loop;
            end if;
        end if;
    end process;

    process (clk)
        variable b, row0, col0, base : natural;
    begin
      if rising_edge(clk) then
        b := to_integer(rblk);
        if rslot = '1' then
            base := 24;
        else
            base := 0;
        end if;
        if b < 16 then
            row0 := (b / 4) * 4;
            col0 := (b mod 4) * 4;
        elsif b < 20 then
            row0 := 16 + ((b - 16) / 2) * 4;
            col0 := ((b - 16) mod 2) * 4;
        else
            row0 := 16 + (((b - 20) mod 4) / 2) * 4;
            col0 := 8 + ((b - 20) mod 2) * 4;
        end if;
        for r in 0 to 3 loop
            for c in 0 to 3 loop
                rdata((r * 4 + c) * 8 + 7 downto (r * 4 + c) * 8) <= mem(base + row0 + r)(col0 + c);
            end loop;
        end loop;
      end if;
    end process;
end architecture;
