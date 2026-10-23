# Timing constraints for h264_dec_top (single clock domain).
# 100 MHz target: about 200 cycles per macroblock give 1920x1080 at ~60 fps.
# If SmartTime reports violations, 75 MHz (13.333 ns) still decodes
# 1920x1080 at 30 fps with margin; faster grades may reach 125 MHz.
create_clock -name {clk} -period 10.000 [ get_ports { clk } ]

# The decoder has no other clocks; all AXI / AXI-Stream / status ports are
# synchronous to clk.  When the core is instantiated inside a SmartDesign the
# clock comes from PF_CCC and Libero derives the constraint automatically
# (derive_constraints); this file is for stand-alone synthesis runs.
