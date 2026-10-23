-- ============================================================================
-- h264_dec_hdmi_top.vhd
-- Complete fabric design for the Video Kit:
--   PC --Ethernet/MSS--> h264_dec_mss_top (link + decoder) --DDR--> h264_disp
--   --TMDS lane words--> PF_XCVR (PMA mode, 4 lanes x 40 bit) --> HDMI
-- One AXI4 master towards the DDR controller: writes from the decoder,
-- reads from the decoder (ARID 0) and the display (ARID 1), joined by
-- h264_axi_rd_arb.  Display registers DISP_CTRL / DISP_STAT live in the APB
-- window of the MSS (offsets 0x2C / 0x30).
--
-- Transceiver connection (as in Microchip's PolarFire HDMI 2.0 reference):
--   o_tmds_clk -> LANE0_TX_DATA, o_tmds_b -> LANE1, o_tmds_g -> LANE2,
--   o_tmds_r -> LANE3; hdmi_clk <- LANE0_TX_CLK_R (lane rate / 40 = pixel
--   clock / 4), hdmi_clk_stable <- LANE0_TX_CLK_STABLE.
-- Default timing: CEA-861 1920x1080p60 (pixel clock 148.5 MHz, lane rate
-- 1485 Mbit/s, hdmi_clk 37.125 MHz).
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity h264_dec_hdmi_top is
    generic (
        G_FB_BASE : std_logic_vector(31 downto 0) := x"00000000";
        G_FIFO_AW : positive := 15;
        G_H_ACT   : positive := 1920;
        G_H_FP    : positive := 88;
        G_H_SYNC  : positive := 44;
        G_H_BP    : positive := 148;
        G_V_ACT   : positive := 1080;
        G_V_FP    : positive := 4;
        G_V_SYNC  : positive := 5;
        G_V_BP    : positive := 36
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;
        -- APB3 slave from the MSS (FIC3, on clk)
        paddr           : in  std_logic_vector(31 downto 0);
        psel            : in  std_logic;
        penable         : in  std_logic;
        pwrite          : in  std_logic;
        pwdata          : in  std_logic_vector(31 downto 0);
        prdata          : out std_logic_vector(31 downto 0);
        pready          : out std_logic;
        pslverr         : out std_logic;
        led_rx          : out std_logic;
        led_err         : out std_logic;
        led_done        : out std_logic;
        -- AXI4 master to the DDR controller
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
        -- HDMI transceiver (PF_XCVR, PMA mode)
        hdmi_clk        : in  std_logic;
        hdmi_clk_stable : in  std_logic;
        o_tmds_clk      : out std_logic_vector(39 downto 0);
        o_tmds_b        : out std_logic_vector(39 downto 0);
        o_tmds_g        : out std_logic_vector(39 downto 0);
        o_tmds_r        : out std_logic_vector(39 downto 0)
    );
end entity;

architecture rtl of h264_dec_hdmi_top is
    signal d_araddr, v_araddr : std_logic_vector(31 downto 0);
    signal d_arlen, v_arlen : std_logic_vector(7 downto 0);
    signal d_arvalid, d_arready, v_arvalid, v_arready : std_logic;
    signal d_rvalid, d_rready, v_rvalid, v_rready : std_logic;
    signal frame_valid, frame_ready, rel : std_logic;
    signal frame_index, rel_index : std_logic_vector(4 downto 0);
    signal addr_y, addr_cb, addr_cr : std_logic_vector(31 downto 0);
    signal width, height : std_logic_vector(11 downto 0);
    signal crop_l, crop_r, crop_t, crop_b : std_logic_vector(10 downto 0);
    signal disp_ctrl, disp_status : std_logic_vector(31 downto 0);
    signal dec_rst, disp_busy : std_logic;
begin
    u_dec : entity work.h264_dec_mss_top
        generic map (G_FB_BASE => G_FB_BASE, G_FIFO_AW => G_FIFO_AW)
        port map (
            clk => clk, rst => rst,
            paddr => paddr, psel => psel, penable => penable, pwrite => pwrite, pwdata => pwdata,
            prdata => prdata, pready => pready, pslverr => pslverr,
            led_rx => led_rx, led_err => led_err, led_done => led_done,
            m_axi_awid => m_axi_awid, m_axi_awaddr => m_axi_awaddr, m_axi_awlen => m_axi_awlen,
            m_axi_awsize => m_axi_awsize, m_axi_awburst => m_axi_awburst, m_axi_awlock => m_axi_awlock,
            m_axi_awcache => m_axi_awcache, m_axi_awprot => m_axi_awprot, m_axi_awqos => m_axi_awqos,
            m_axi_awvalid => m_axi_awvalid, m_axi_awready => m_axi_awready,
            m_axi_wdata => m_axi_wdata, m_axi_wstrb => m_axi_wstrb, m_axi_wlast => m_axi_wlast,
            m_axi_wvalid => m_axi_wvalid, m_axi_wready => m_axi_wready,
            m_axi_bid => m_axi_bid, m_axi_bresp => m_axi_bresp, m_axi_bvalid => m_axi_bvalid,
            m_axi_bready => m_axi_bready,
            m_axi_arid => open, m_axi_araddr => d_araddr, m_axi_arlen => d_arlen,
            m_axi_arsize => m_axi_arsize, m_axi_arburst => m_axi_arburst, m_axi_arlock => m_axi_arlock,
            m_axi_arcache => m_axi_arcache, m_axi_arprot => m_axi_arprot, m_axi_arqos => m_axi_arqos,
            m_axi_arvalid => d_arvalid, m_axi_arready => d_arready,
            m_axi_rid => "0000", m_axi_rdata => m_axi_rdata, m_axi_rresp => m_axi_rresp,
            m_axi_rlast => m_axi_rlast, m_axi_rvalid => d_rvalid, m_axi_rready => d_rready,
            o_frame_valid => frame_valid, o_frame_index => frame_index,
            o_frame_addr_y => addr_y, o_frame_addr_cb => addr_cb, o_frame_addr_cr => addr_cr,
            i_frame_ready => frame_ready, i_release => rel, i_release_index => rel_index,
            o_width => width, o_height => height, o_crop_left => crop_l, o_crop_right => crop_r,
            o_crop_top => crop_t, o_crop_bottom => crop_b,
            o_error => open, o_stream_done => open, o_pic_count => open,
            o_disp_ctrl => disp_ctrl, i_disp_status => disp_status, o_dec_rst => dec_rst,
            i_disp_busy => disp_busy);

    -- decoder and display reads share the read port (same size / burst type)
    u_arb : entity work.h264_axi_rd_arb
        port map (
            clk => clk, rst => rst,
            s0_araddr => d_araddr, s0_arlen => d_arlen, s0_arvalid => d_arvalid, s0_arready => d_arready,
            s0_rvalid => d_rvalid, s0_rready => d_rready,
            s1_araddr => v_araddr, s1_arlen => v_arlen, s1_arvalid => v_arvalid, s1_arready => v_arready,
            s1_rvalid => v_rvalid, s1_rready => v_rready,
            m_arid => m_axi_arid, m_araddr => m_axi_araddr, m_arlen => m_axi_arlen,
            m_arvalid => m_axi_arvalid, m_arready => m_axi_arready,
            m_rid => m_axi_rid, m_rvalid => m_axi_rvalid, m_rready => m_axi_rready);

    u_disp : entity work.h264_disp
        generic map (G_H_ACT => G_H_ACT, G_H_FP => G_H_FP, G_H_SYNC => G_H_SYNC, G_H_BP => G_H_BP,
                     G_V_ACT => G_V_ACT, G_V_FP => G_V_FP, G_V_SYNC => G_V_SYNC, G_V_BP => G_V_BP)
        port map (
            clk => clk, rst => rst, i_ctrl => disp_ctrl, o_status => disp_status, i_dec_rst => dec_rst,
            i_frame_valid => frame_valid, i_frame_index => frame_index,
            i_frame_addr_y => addr_y, i_frame_addr_cb => addr_cb, i_frame_addr_cr => addr_cr,
            o_frame_ready => frame_ready, o_release => rel, o_release_index => rel_index,
            i_width => width, i_height => height, i_crop_left => crop_l, i_crop_right => crop_r,
            i_crop_top => crop_t, i_crop_bottom => crop_b,
            m_araddr => v_araddr, m_arlen => v_arlen, m_arvalid => v_arvalid, m_arready => v_arready,
            m_rdata => m_axi_rdata, m_rlast => m_axi_rlast, m_rvalid => v_rvalid, m_rready => v_rready,
            px_clk => hdmi_clk, px_clk_stable => hdmi_clk_stable,
            o_tmds_clk => o_tmds_clk, o_tmds_b => o_tmds_b, o_tmds_g => o_tmds_g, o_tmds_r => o_tmds_r,
            o_busy => disp_busy);
end architecture;
