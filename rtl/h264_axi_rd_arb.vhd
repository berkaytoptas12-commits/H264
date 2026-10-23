-- ============================================================================
-- h264_axi_rd_arb.vhd
-- Joins two AXI4 read masters on one read port: the decoder (s0, ARID 0) and
-- the display reader (s1, ARID 1).  Round robin on AR, one request per clock;
-- read data is routed back by RID bit 0, so the slave may interleave the two
-- IDs.  Each master's own requests keep their order (one ID per master).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_axi_rd_arb is
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        s0_araddr  : in  std_logic_vector(31 downto 0);
        s0_arlen   : in  std_logic_vector(7 downto 0);
        s0_arvalid : in  std_logic;
        s0_arready : out std_logic;
        s0_rvalid  : out std_logic;
        s0_rready  : in  std_logic;
        s1_araddr  : in  std_logic_vector(31 downto 0);
        s1_arlen   : in  std_logic_vector(7 downto 0);
        s1_arvalid : in  std_logic;
        s1_arready : out std_logic;
        s1_rvalid  : out std_logic;
        s1_rready  : in  std_logic;
        m_arid     : out std_logic_vector(3 downto 0);
        m_araddr   : out std_logic_vector(31 downto 0);
        m_arlen    : out std_logic_vector(7 downto 0);
        m_arvalid  : out std_logic;
        m_arready  : in  std_logic;
        m_rid      : in  std_logic_vector(3 downto 0);
        m_rvalid   : in  std_logic;
        m_rready   : out std_logic
    );
end entity;

architecture rtl of h264_axi_rd_arb is
    signal arvalid : std_logic;
    signal last0   : std_logic;             -- last grant went to s0
    signal can, pick1 : std_logic;
begin
    can <= not arvalid or m_arready;
    pick1 <= s1_arvalid and (not s0_arvalid or last0);
    s0_arready <= can and s0_arvalid and not pick1;
    s1_arready <= can and pick1;
    m_arvalid <= arvalid;

    s0_rvalid <= m_rvalid and not m_rid(0);
    s1_rvalid <= m_rvalid and m_rid(0);
    m_rready <= s1_rready when m_rid(0) = '1' else s0_rready;

    process (clk)
    begin
        if rising_edge(clk) then
            if can = '1' then
                arvalid <= s0_arvalid or s1_arvalid;
                if pick1 = '1' then
                    m_araddr <= s1_araddr;
                    m_arlen <= s1_arlen;
                    m_arid <= "0001";
                    last0 <= '0';
                elsif s0_arvalid = '1' then
                    m_araddr <= s0_araddr;
                    m_arlen <= s0_arlen;
                    m_arid <= "0000";
                    last0 <= '1';
                end if;
            end if;
            if rst = '1' then
                arvalid <= '0';
                last0 <= '0';
            end if;
        end if;
    end process;
end architecture;
