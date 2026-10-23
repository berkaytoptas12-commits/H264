# Extra timing constraints for h264_dec_hdmi_top (stand-alone synthesis).
# hdmi_clk = transceiver TX clock: 1080p60 -> 1485 Mbit/s / 40 = 37.125 MHz
# (720p60 -> 18.5625 MHz).  In the kit's SmartDesign Libero derives it from
# PF_XCVR (derive_constraints).
create_clock -name {hdmi_clk} -period 26.936 [ get_ports { hdmi_clk } ]

# The two domains meet only in h264_disp: dual clock line RAMs, Gray coded
# line counters with two-flop synchronisers, quasi static slot meta data.
set_clock_groups -asynchronous -group [ get_clocks { clk } ] -group [ get_clocks { hdmi_clk } ]
