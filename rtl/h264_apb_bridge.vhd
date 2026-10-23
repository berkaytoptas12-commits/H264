-- ============================================================================
-- h264_apb_bridge.vhd
-- APB slave (MSS FIC3) that gives the MSS processor a byte channel to
-- h264_link: the firmware writes the bytes of received UDP datagrams (link
-- frames) and reads the link responses back.  Zero wait states.
--
-- Registers (byte offsets, 32 bit):
--   00 ID        RO  0x48323634 ("H264")
--   04 CAPS      RO  [15:0] TX FIFO entries, [31:16] RX FIFO bytes
--   08 TXFREE    RO  free TX FIFO entries
--   0C TXDATA    WO  one byte  [7:0]                      (one entry)
--   10 TXDATA4   WO  four bytes, [7:0] first .. [31:24] last (one entry)
--   14 RXCOUNT   RO  bytes in the RX FIFO
--   18 RXDATA    RO  [8] valid, [7:0] byte; a read with valid = 1 pops it
--   1C STATUS    RO  [0] decoder error, [1] stream done
--   20 PICTURES  RO  decoded pictures
--   24 SIZE      RO  [11:0] width, [27:16] height (luma samples)
--   28 CTRL      WO  [0] = 1: reset link and decoder
--   2C DISP_CTRL RW  display (h264_disp): [0] off, [1] colour bars, [2] bit
--                    reversed lanes, [31:16] stream frame rate * 100 (3000)
--   30 DISP_STAT RO  [15:0] frame switches, [31:16] lines shown black (late data)
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_apb_bridge is
    generic (
        G_TX_AW : positive := 9;            -- TX FIFO 2**G_TX_AW entries of 1..4 bytes
        G_RX_AW : positive := 6             -- RX FIFO 2**G_RX_AW bytes
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;
        -- APB3 slave
        paddr       : in  std_logic_vector(11 downto 0);
        psel        : in  std_logic;
        penable     : in  std_logic;
        pwrite      : in  std_logic;
        pwdata      : in  std_logic_vector(31 downto 0);
        prdata      : out std_logic_vector(31 downto 0);
        pready      : out std_logic;
        pslverr     : out std_logic;
        -- byte channel to h264_link
        o_rx_data   : out std_logic_vector(7 downto 0);
        o_rx_valid  : out std_logic;
        i_tx_data   : in  std_logic_vector(7 downto 0);
        i_tx_valid  : in  std_logic;
        o_tx_ready  : out std_logic;
        -- decoder status
        i_pic_count : in  std_logic_vector(31 downto 0);
        i_error     : in  std_logic;
        i_stream_done : in  std_logic;
        i_width     : in  std_logic_vector(11 downto 0);
        i_height    : in  std_logic_vector(11 downto 0);
        o_soft_rst  : out std_logic;        -- CTRL[0] written (16 cycles)
        -- display control / status
        o_disp_ctrl : out std_logic_vector(31 downto 0);
        i_disp_status : in std_logic_vector(31 downto 0) := (others => '0')
    );
end entity;

architecture rtl of h264_apb_bridge is
    constant TXN : natural := 2 ** G_TX_AW;
    constant RXN : natural := 2 ** G_RX_AW;
    subtype txp_t is unsigned(G_TX_AW downto 0);
    subtype rxp_t is unsigned(G_RX_AW downto 0);

    -- TX FIFO: entry = byte count - 1 (2 bits) & 4 bytes
    signal tx_we    : std_logic;
    signal tx_wd    : std_logic_vector(33 downto 0);
    signal tx_ra    : unsigned(G_TX_AW - 1 downto 0);
    signal tx_rd    : std_logic_vector(33 downto 0);
    signal tx_wp, tx_rp : txp_t;
    signal txr_v1, txr_v2 : std_logic;          -- RAM read in flight
    signal ser_v    : std_logic;                -- entry being serialised
    signal ser_d    : std_logic_vector(31 downto 0);
    signal ser_n    : unsigned(1 downto 0);     -- bytes left - 1
    signal ent_v    : std_logic;                -- next entry waiting
    signal ent_d    : std_logic_vector(33 downto 0);

    -- RX FIFO (registers, head readable without wait)
    type rx_mem_t is array (0 to RXN - 1) of std_logic_vector(7 downto 0);
    signal rx_mem   : rx_mem_t;
    signal rx_wp, rx_rp : rxp_t;
    signal rx_cnt   : rxp_t;

    signal srst_cnt : natural range 0 to 16;
    signal rd_acc, wr_acc : std_logic;
    signal tx_free  : txp_t;
    signal disp_ctrl : std_logic_vector(31 downto 0);
begin
    o_disp_ctrl <= disp_ctrl;
    u_txram : entity work.h264_ram_sdp
        generic map (G_AW => G_TX_AW, G_DW => 34)
        port map (clk => clk, we => tx_we, waddr => tx_wp(G_TX_AW - 1 downto 0), wdata => tx_wd,
                  raddr => tx_ra, rdata => tx_rd);

    pready <= '1';
    pslverr <= '0';
    rd_acc <= psel and penable and not pwrite;
    wr_acc <= psel and penable and pwrite;
    rx_cnt <= rx_wp - rx_rp;
    tx_free <= to_unsigned(TXN, G_TX_AW + 1) - (tx_wp - tx_rp);
    o_tx_ready <= '1' when rx_cnt /= RXN else '0';
    o_soft_rst <= '1' when srst_cnt /= 0 else '0';

    -- read data
    process (all)
        variable r : std_logic_vector(31 downto 0);
    begin
        r := (others => '0');
        case paddr(7 downto 2) is
            when "000000" => r := x"48323634";
            when "000001" =>
                r(15 downto 0) := std_logic_vector(to_unsigned(TXN, 16));
                r(31 downto 16) := std_logic_vector(to_unsigned(RXN, 16));
            when "000010" => r(G_TX_AW downto 0) := std_logic_vector(tx_free);
            when "000101" => r(G_RX_AW downto 0) := std_logic_vector(rx_cnt);
            when "000110" =>
                if rx_cnt /= 0 then
                    r(8) := '1';
                    r(7 downto 0) := rx_mem(to_integer(rx_rp(G_RX_AW - 1 downto 0)));
                end if;
            when "000111" =>
                r(0) := i_error;
                r(1) := i_stream_done;
            when "001000" => r := i_pic_count;
            when "001001" =>
                r(11 downto 0) := i_width;
                r(27 downto 16) := i_height;
            when "001011" => r := disp_ctrl;
            when "001100" => r := i_disp_status;
            when others => null;
        end case;
        prdata <= r;
    end process;

    process (clk)
        variable issue : boolean;
        variable flush : boolean;
    begin
        if rising_edge(clk) then
            tx_we <= '0';
            o_rx_valid <= '0';
            flush := false;
            if srst_cnt /= 0 then
                srst_cnt <= srst_cnt - 1;
            end if;

            -- APB writes
            if wr_acc = '1' then
                case paddr(7 downto 2) is
                    when "000011" =>
                        if tx_free /= 0 then
                            tx_we <= '1';
                            tx_wd <= "00" & x"000000" & pwdata(7 downto 0);
                        end if;
                    when "000100" =>
                        if tx_free /= 0 then
                            tx_we <= '1';
                            tx_wd <= "11" & pwdata;
                        end if;
                    when "001010" =>
                        if pwdata(0) = '1' then
                            srst_cnt <= 16;
                            flush := true;
                        end if;
                    when "001011" =>
                        disp_ctrl <= pwdata;
                    when others => null;
                end case;
            end if;
            if tx_we = '1' then
                tx_wp <= tx_wp + 1;
            end if;

            -- TX FIFO -> serialiser (2 cycle RAM latency, one waiting entry)
            issue := tx_wp /= tx_rp and txr_v1 = '0' and txr_v2 = '0' and ent_v = '0';
            txr_v1 <= '0';
            if issue then
                tx_ra <= tx_rp(G_TX_AW - 1 downto 0);
                tx_rp <= tx_rp + 1;
                txr_v1 <= '1';
            end if;
            txr_v2 <= txr_v1;
            if txr_v2 = '1' then
                ent_v <= '1';
                ent_d <= tx_rd;
            end if;
            if ser_v = '1' then
                o_rx_valid <= '1';
                o_rx_data <= ser_d(7 downto 0);
                ser_d <= x"00" & ser_d(31 downto 8);
                if ser_n = 0 then
                    ser_v <= '0';
                end if;
                ser_n <= ser_n - 1;
            end if;
            if (ser_v = '0' or ser_n = 0) and ent_v = '1' then
                ser_v <= '1';
                ser_d <= ent_d(31 downto 0);
                ser_n <= unsigned(ent_d(33 downto 32));
                ent_v <= '0';
            end if;

            -- RX FIFO
            if i_tx_valid = '1' and rx_cnt /= RXN then
                rx_mem(to_integer(rx_wp(G_RX_AW - 1 downto 0))) <= i_tx_data;
                rx_wp <= rx_wp + 1;
            end if;
            if rd_acc = '1' and paddr(7 downto 2) = "000110" and rx_cnt /= 0 then
                rx_rp <= rx_rp + 1;
            end if;

            if rst = '1' or flush then
                tx_we <= '0';
                tx_wp <= (others => '0');
                tx_rp <= (others => '0');
                txr_v1 <= '0';
                txr_v2 <= '0';
                ent_v <= '0';
                ser_v <= '0';
                rx_wp <= (others => '0');
                rx_rp <= (others => '0');
            end if;
            if rst = '1' then
                srst_cnt <= 0;
                disp_ctrl <= x"0BB80000";      -- 30 fps, display on
            end if;
        end if;
    end process;
end architecture;
