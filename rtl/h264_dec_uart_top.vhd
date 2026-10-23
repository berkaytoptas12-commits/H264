-- ============================================================================
-- h264_dec_uart_top.vhd
-- Decoder with the PC stream link over a fabric UART (no MSS):
--   uart_rx / uart_tx -> h264_uart_rx / _tx -> h264_link -> h264_dec_top
-- The host program (host/h264_send.py, host/h264_gui.py) sends the H.264
-- Annex-B file in CRC protected frames with credit based flow control,
-- START resets the decoder, END marks the end of the stream.
-- G_CLK_HZ / G_BAUD must give an integer close to the real ratio (2 % max).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_dec_uart_top is
    generic (
        G_FB_BASE : std_logic_vector(31 downto 0) := x"00000000";
        G_CLK_HZ  : positive := 100_000_000;
        G_BAUD    : positive := 2_000_000;
        G_FIFO_AW : positive := 15           -- 32 KiB receive FIFO
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;                     -- synchronous, active high
        -- serial link to the PC (8N1)
        uart_rx         : in  std_logic;
        uart_tx         : out std_logic;
        -- LEDs (active high): frames arriving, link or decoder error, stream done
        led_rx          : out std_logic;
        led_err         : out std_logic;
        led_done        : out std_logic;
        -- AXI4 master
        m_axi_awid      : out std_logic_vector(3 downto 0);
        m_axi_awaddr    : out std_logic_vector(31 downto 0);
        m_axi_awlen     : out std_logic_vector(7 downto 0);
        m_axi_awsize    : out std_logic_vector(2 downto 0);
        m_axi_awburst   : out std_logic_vector(1 downto 0);
        m_axi_awlock    : out std_logic;
        m_axi_awcache   : out std_logic_vector(3 downto 0);
        m_axi_awprot    : out std_logic_vector(2 downto 0);
        m_axi_awqos     : out std_logic_vector(3 downto 0);
        m_axi_awvalid   : out std_logic;
        m_axi_awready   : in  std_logic;
        m_axi_wdata     : out std_logic_vector(63 downto 0);
        m_axi_wstrb     : out std_logic_vector(7 downto 0);
        m_axi_wlast     : out std_logic;
        m_axi_wvalid    : out std_logic;
        m_axi_wready    : in  std_logic;
        m_axi_bid       : in  std_logic_vector(3 downto 0);
        m_axi_bresp     : in  std_logic_vector(1 downto 0);
        m_axi_bvalid    : in  std_logic;
        m_axi_bready    : out std_logic;
        m_axi_arid      : out std_logic_vector(3 downto 0);
        m_axi_araddr    : out std_logic_vector(31 downto 0);
        m_axi_arlen     : out std_logic_vector(7 downto 0);
        m_axi_arsize    : out std_logic_vector(2 downto 0);
        m_axi_arburst   : out std_logic_vector(1 downto 0);
        m_axi_arlock    : out std_logic;
        m_axi_arcache   : out std_logic_vector(3 downto 0);
        m_axi_arprot    : out std_logic_vector(2 downto 0);
        m_axi_arqos     : out std_logic_vector(3 downto 0);
        m_axi_arvalid   : out std_logic;
        m_axi_arready   : in  std_logic;
        m_axi_rid       : in  std_logic_vector(3 downto 0);
        m_axi_rdata     : in  std_logic_vector(63 downto 0);
        m_axi_rresp     : in  std_logic_vector(1 downto 0);
        m_axi_rlast     : in  std_logic;
        m_axi_rvalid    : in  std_logic;
        m_axi_rready    : out std_logic;
        -- decoded frames
        o_frame_valid   : out std_logic;
        o_frame_index   : out std_logic_vector(4 downto 0);
        o_frame_addr_y  : out std_logic_vector(31 downto 0);
        o_frame_addr_cb : out std_logic_vector(31 downto 0);
        o_frame_addr_cr : out std_logic_vector(31 downto 0);
        i_frame_ready   : in  std_logic;
        i_release       : in  std_logic;
        i_release_index : in  std_logic_vector(4 downto 0);
        -- picture geometry (active SPS); luma samples, crop in luma samples
        o_width         : out std_logic_vector(11 downto 0);
        o_height        : out std_logic_vector(11 downto 0);
        o_crop_left     : out std_logic_vector(10 downto 0);
        o_crop_right    : out std_logic_vector(10 downto 0);
        o_crop_top      : out std_logic_vector(10 downto 0);
        o_crop_bottom   : out std_logic_vector(10 downto 0);
        -- status
        o_error         : out std_logic;     -- unsupported or corrupt stream (sticky)
        o_stream_done   : out std_logic;     -- end of stream reached, all frames delivered
        o_pic_count     : out std_logic_vector(31 downto 0)
    );
end entity;

architecture rtl of h264_dec_uart_top is
    constant DIV : positive := (G_CLK_HZ + G_BAUD / 2) / G_BAUD;

    signal rx_data, tx_data : std_logic_vector(7 downto 0);
    signal rx_valid, tx_valid, tx_ready : std_logic;
    signal s_tdata  : std_logic_vector(7 downto 0);
    signal s_tvalid, s_tlast, s_tready : std_logic;
    signal dec_rst, dec_rst_any : std_logic;
    signal frame_ok, frame_bad : std_logic;
    signal pic_cnt  : std_logic_vector(31 downto 0);
    signal dec_err, dec_done : std_logic;
    signal width_s, height_s : std_logic_vector(11 downto 0);
    signal rx_stretch, bad_stretch : unsigned(22 downto 0);
begin
    u_rx : entity work.h264_uart_rx
        generic map (G_DIV => DIV)
        port map (clk => clk, rst => rst, i_rx => uart_rx, o_data => rx_data, o_valid => rx_valid, o_ferr => open);

    u_tx : entity work.h264_uart_tx
        generic map (G_DIV => DIV)
        port map (clk => clk, rst => rst, i_data => tx_data, i_valid => tx_valid, o_ready => tx_ready, o_tx => uart_tx);

    u_link : entity work.h264_link
        generic map (G_FIFO_AW => G_FIFO_AW)
        port map (clk => clk, rst => rst,
                  i_rx_data => rx_data, i_rx_valid => rx_valid,
                  o_tx_data => tx_data, o_tx_valid => tx_valid, i_tx_ready => tx_ready,
                  m_axis_tdata => s_tdata, m_axis_tvalid => s_tvalid, m_axis_tlast => s_tlast,
                  m_axis_tready => s_tready, o_dec_rst => dec_rst,
                  i_pic_count => pic_cnt, i_error => dec_err, i_stream_done => dec_done,
                  i_width => width_s, i_height => height_s,
                  o_frame_ok => frame_ok, o_frame_bad => frame_bad);

    dec_rst_any <= rst or dec_rst;

    u_dec : entity work.h264_dec_top
        generic map (G_FB_BASE => G_FB_BASE)
        port map (
            clk => clk,
            rst => dec_rst_any,
            s_axis_tdata => s_tdata,
            s_axis_tvalid => s_tvalid,
            s_axis_tlast => s_tlast,
            s_axis_tready => s_tready,
            m_axi_awid => m_axi_awid,
            m_axi_awaddr => m_axi_awaddr,
            m_axi_awlen => m_axi_awlen,
            m_axi_awsize => m_axi_awsize,
            m_axi_awburst => m_axi_awburst,
            m_axi_awlock => m_axi_awlock,
            m_axi_awcache => m_axi_awcache,
            m_axi_awprot => m_axi_awprot,
            m_axi_awqos => m_axi_awqos,
            m_axi_awvalid => m_axi_awvalid,
            m_axi_awready => m_axi_awready,
            m_axi_wdata => m_axi_wdata,
            m_axi_wstrb => m_axi_wstrb,
            m_axi_wlast => m_axi_wlast,
            m_axi_wvalid => m_axi_wvalid,
            m_axi_wready => m_axi_wready,
            m_axi_bid => m_axi_bid,
            m_axi_bresp => m_axi_bresp,
            m_axi_bvalid => m_axi_bvalid,
            m_axi_bready => m_axi_bready,
            m_axi_arid => m_axi_arid,
            m_axi_araddr => m_axi_araddr,
            m_axi_arlen => m_axi_arlen,
            m_axi_arsize => m_axi_arsize,
            m_axi_arburst => m_axi_arburst,
            m_axi_arlock => m_axi_arlock,
            m_axi_arcache => m_axi_arcache,
            m_axi_arprot => m_axi_arprot,
            m_axi_arqos => m_axi_arqos,
            m_axi_arvalid => m_axi_arvalid,
            m_axi_arready => m_axi_arready,
            m_axi_rid => m_axi_rid,
            m_axi_rdata => m_axi_rdata,
            m_axi_rresp => m_axi_rresp,
            m_axi_rlast => m_axi_rlast,
            m_axi_rvalid => m_axi_rvalid,
            m_axi_rready => m_axi_rready,
            o_frame_valid => o_frame_valid,
            o_frame_index => o_frame_index,
            o_frame_addr_y => o_frame_addr_y,
            o_frame_addr_cb => o_frame_addr_cb,
            o_frame_addr_cr => o_frame_addr_cr,
            i_frame_ready => i_frame_ready,
            i_release => i_release,
            i_release_index => i_release_index,
            o_width => width_s,
            o_height => height_s,
            o_crop_left => o_crop_left,
            o_crop_right => o_crop_right,
            o_crop_top => o_crop_top,
            o_crop_bottom => o_crop_bottom,
            o_error => dec_err,
            o_stream_done => dec_done,
            o_pic_count => pic_cnt
        );

    o_width <= width_s;
    o_height <= height_s;
    o_error <= dec_err;
    o_stream_done <= dec_done;
    o_pic_count <= pic_cnt;

    -- LEDs: about 80 ms pulse stretching at 100 MHz
    process (clk)
    begin
        if rising_edge(clk) then
            if frame_ok = '1' then
                rx_stretch <= (others => '1');
            elsif rx_stretch /= 0 then
                rx_stretch <= rx_stretch - 1;
            end if;
            if frame_bad = '1' then
                bad_stretch <= (others => '1');
            elsif bad_stretch /= 0 then
                bad_stretch <= bad_stretch - 1;
            end if;
            if rst = '1' then
                rx_stretch <= (others => '0');
                bad_stretch <= (others => '0');
            end if;
        end if;
    end process;
    led_rx <= '1' when rx_stretch /= 0 else '0';
    led_err <= '1' when bad_stretch /= 0 or dec_err = '1' else '0';
    led_done <= dec_done;
end architecture;
