-- ============================================================================
-- h264_dec_top.vhd
-- H.264 / AVC Constrained Baseline decoder, fabric only.
--
--   AXI4-Stream (8 bit) Annex-B byte stream in, tlast = end of stream
--   AXI4 master (32 bit address, 64 bit data) to the frame store
--     read channel  : motion compensation reference fetch
--     write channel : deblocked picture output
--   decoded frames are announced in decoding order on o_frame_*; the
--   buffer is owned by the consumer until it is returned with i_release.
--
-- Pipeline (one macroblock token per stage):
--   nal -> bitreader -> hdr / mbparse -> mc -> recon -> deblock
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.h264_pkg.all;

entity h264_dec_top is
    generic (
        G_FB_BASE : std_logic_vector(31 downto 0) := x"00000000"   -- frame store base (4 MiB aligned)
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;                     -- synchronous, active high
        -- bitstream input
        s_axis_tdata    : in  std_logic_vector(7 downto 0);
        s_axis_tvalid   : in  std_logic;
        s_axis_tlast    : in  std_logic;
        s_axis_tready   : out std_logic;
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

architecture rtl of h264_dec_top is
    signal n_data   : std_logic_vector(7 downto 0);
    signal n_first, n_last, n_eos, n_valid, n_ready : std_logic;

    signal br_win    : std_logic_vector(63 downto 0);
    signal br_avail32, br_avail64, br_more, br_active, br_eos : std_logic;
    signal br_bitpos : unsigned(31 downto 0);
    signal br_consume, hdr_consume, mbp_consume : unsigned(6 downto 0);
    signal br_flush, hdr_own : std_logic;

    signal slice : slice_t;
    signal slice_start, slice_done : std_logic;
    signal hdr_err, mbp_err : std_logic;

    signal cw_en : std_logic;
    signal cw_addr, cr_addr : unsigned(8 downto 0);
    signal cw_data, cr_data : std_logic_vector(63 downto 0);
    signal coef_free, pred_free, rec_free : std_logic;

    signal t_p, t_m, t_r : mbinfo_t;          -- tokens parser->mc, mc->recon, recon->deblock
    signal t_p_v, t_m_v, t_r_v : std_logic;
    signal mc_rdy, rc_rdy, db_rdy : std_logic;

    signal pw_en, rw_en : std_logic_vector(15 downto 0);
    signal pw_addr, pr_addr, rw_addr, rr_addr : unsigned(5 downto 0);
    signal pw_data, pr_data, rw_data, rr_data : std_logic_vector(127 downto 0);

    signal pic_done : std_logic;
    signal pic_fb   : unsigned(4 downto 0);
    signal frame_fb : unsigned(4 downto 0);
    signal mb_w, mb_h : unsigned(7 downto 0);
    signal crop_l, crop_r, crop_t, crop_b : unsigned(9 downto 0);
    signal pic_count : unsigned(31 downto 0);
begin
    u_nal : entity work.h264_nal
        port map (clk => clk, rst => rst, s_tdata => s_axis_tdata, s_tvalid => s_axis_tvalid,
                  s_tlast => s_axis_tlast, s_tready => s_axis_tready, o_data => n_data, o_first => n_first,
                  o_last => n_last, o_eos => n_eos, o_valid => n_valid, i_ready => n_ready);

    br_consume <= hdr_consume when hdr_own = '1' else mbp_consume;

    u_br : entity work.h264_bitreader
        port map (clk => clk, rst => rst, i_data => n_data, i_first => n_first, i_last => n_last,
                  i_eos => n_eos, i_valid => n_valid, o_ready => n_ready, o_win => br_win, o_nbits => open,
                  o_avail32 => br_avail32, o_avail64 => br_avail64, o_more => br_more, o_active => br_active,
                  o_eos => br_eos, o_bitpos => br_bitpos, i_consume => br_consume, i_flush => br_flush);

    u_hdr : entity work.h264_hdr
        port map (clk => clk, rst => rst, br_win => br_win, br_avail64 => br_avail64, br_more => br_more,
                  br_active => br_active, br_eos => br_eos, hdr_consume => hdr_consume, br_flush => br_flush,
                  own_br => hdr_own, slice => slice, slice_start => slice_start, slice_done => slice_done,
                  pic_done => pic_done, pic_done_fb => pic_fb, o_frame_valid => o_frame_valid,
                  o_frame_fb => frame_fb, i_frame_ready => i_frame_ready, i_release => i_release,
                  i_release_fb => unsigned(i_release_index), o_mb_w => mb_w, o_mb_h => mb_h,
                  o_crop_l => crop_l, o_crop_r => crop_r, o_crop_t => crop_t, o_crop_b => crop_b,
                  o_error => hdr_err, o_stream_done => o_stream_done, o_pic_count => pic_count);

    u_mbp : entity work.h264_mbparse
        port map (clk => clk, rst => rst, br_win => br_win, br_avail32 => br_avail32, br_more => br_more,
                  br_bitpos => br_bitpos, mbp_consume => mbp_consume, slice => slice,
                  slice_start => slice_start, slice_done => slice_done, o_error => mbp_err,
                  cw_en => cw_en, cw_addr => cw_addr, cw_data => cw_data, i_coef_free => coef_free,
                  o_mb => t_p, o_mb_valid => t_p_v, i_mb_ready => mc_rdy);

    u_coef : entity work.h264_ram_sdp
        generic map (G_AW => 9, G_DW => 64)
        port map (clk => clk, we => cw_en, waddr => cw_addr, wdata => cw_data, raddr => cr_addr, rdata => cr_data);

    u_mc : entity work.h264_mc
        generic map (G_FB_BASE => G_FB_BASE)
        port map (clk => clk, rst => rst, i_mb => t_p, i_mb_valid => t_p_v, o_mb_ready => mc_rdy,
                  o_mb => t_m, o_mb_valid => t_m_v, i_mb_ready => rc_rdy, i_pred_free => pred_free,
                  i_pic_done => pic_done, pw_en => pw_en, pw_addr => pw_addr, pw_data => pw_data,
                  m_araddr => m_axi_araddr, m_arlen => m_axi_arlen, m_arsize => m_axi_arsize,
                  m_arburst => m_axi_arburst, m_arvalid => m_axi_arvalid, m_arready => m_axi_arready,
                  m_rdata => m_axi_rdata, m_rlast => m_axi_rlast, m_rvalid => m_axi_rvalid,
                  m_rready => m_axi_rready);

    u_pred : entity work.h264_pixbuf
        generic map (G_AW => 6)
        port map (clk => clk, we => pw_en, waddr => pw_addr, wdata => pw_data, raddr => pr_addr, rdata => pr_data);

    u_rec : entity work.h264_recon
        port map (clk => clk, rst => rst, i_mb => t_m, i_mb_valid => t_m_v, o_mb_ready => rc_rdy,
                  cr_addr => cr_addr, cr_data => cr_data, o_coef_free => coef_free,
                  pr_addr => pr_addr, pr_data => pr_data, o_pred_free => pred_free,
                  rw_en => rw_en, rw_addr => rw_addr, rw_data => rw_data, i_rec_free => rec_free,
                  o_mb => t_r, o_mb_valid => t_r_v, i_mb_ready => db_rdy);

    u_recbuf : entity work.h264_pixbuf
        generic map (G_AW => 6)
        port map (clk => clk, we => rw_en, waddr => rw_addr, wdata => rw_data, raddr => rr_addr, rdata => rr_data);

    u_db : entity work.h264_deblock
        generic map (G_FB_BASE => G_FB_BASE)
        port map (clk => clk, rst => rst, i_mb => t_r, i_mb_valid => t_r_v, o_mb_ready => db_rdy,
                  rr_addr => rr_addr, rr_data => rr_data, o_rec_free => rec_free,
                  o_pic_done => pic_done, o_pic_fb => pic_fb,
                  m_awaddr => m_axi_awaddr, m_awlen => m_axi_awlen, m_awsize => m_axi_awsize,
                  m_awburst => m_axi_awburst, m_awvalid => m_axi_awvalid, m_awready => m_axi_awready,
                  m_wdata => m_axi_wdata, m_wstrb => m_axi_wstrb, m_wlast => m_axi_wlast,
                  m_wvalid => m_axi_wvalid, m_wready => m_axi_wready, m_bvalid => m_axi_bvalid,
                  m_bready => m_axi_bready);

    m_axi_awid <= (others => '0');
    m_axi_arid <= (others => '0');
    m_axi_awlock <= '0';
    m_axi_arlock <= '0';
    m_axi_awcache <= "0011";
    m_axi_arcache <= "0011";
    m_axi_awprot <= "000";
    m_axi_arprot <= "000";
    m_axi_awqos <= "0000";
    m_axi_arqos <= "0000";

    o_frame_index <= std_logic_vector(frame_fb);
    o_frame_addr_y <= std_logic_vector(fb_plane_addr(unsigned(G_FB_BASE), frame_fb, 0, to_unsigned(0, 11), to_unsigned(0, 11)));
    o_frame_addr_cb <= std_logic_vector(fb_plane_addr(unsigned(G_FB_BASE), frame_fb, 1, to_unsigned(0, 11), to_unsigned(0, 11)));
    o_frame_addr_cr <= std_logic_vector(fb_plane_addr(unsigned(G_FB_BASE), frame_fb, 2, to_unsigned(0, 11), to_unsigned(0, 11)));
    o_width <= std_logic_vector(resize(mb_w * 16, 12));
    o_height <= std_logic_vector(resize(mb_h * 16, 12));
    o_crop_left <= std_logic_vector(resize(crop_l * 2, 11));
    o_crop_right <= std_logic_vector(resize(crop_r * 2, 11));
    o_crop_top <= std_logic_vector(resize(crop_t * 2, 11));
    o_crop_bottom <= std_logic_vector(resize(crop_b * 2, 11));
    o_error <= hdr_err or mbp_err;
    o_pic_count <= std_logic_vector(pic_count);
end architecture;
