# Timing constraints for h264_dec_top (single clock domain).
# 150 MHz target; relax to 125 MHz (8.0 ns) if timing does not close.
create_clock -name {clk} -period 6.667 [ get_ports { clk } ]

# The decoder has no other clocks; all AXI / AXI-Stream / status ports are
# synchronous to clk.  When the core is instantiated inside a SmartDesign the
# clock comes from PF_CCC and Libero derives the constraint automatically
# (derive_constraints); this file is for stand-alone synthesis runs.
