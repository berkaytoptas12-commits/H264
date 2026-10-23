# ============================================================================
# hdmi_phy.tcl - HDMI transmitter PHY for h264_dec_hdmi_top (Video Kit:
# SerDes 2 -> PI3HDX1204 re-driver -> HDMI TX connector, 148.5 MHz
# reference oscillator).  Source it in the kit's Libero project:
#
#   source <repo>/libero/hdmi_phy.tcl
#
# It creates PF_XCVR_REF_CLK_HDMI, PF_TX_PLL_HDMI, PF_XCVR_HDMI (PMA mode,
# 4 lanes, 40 bit TX fabric interface, lane rate 10 x pixel clock) and the
# SmartDesign HDMI_PHY:
#   LANE0..3_TX_DATA[39:0] in  <- o_tmds_clk / _b / _g / _r of h264_dec_hdmi_top
#   TX_CLK, TX_CLK_STABLE   out -> hdmi_clk, hdmi_clk_stable
#   REF_CLK_PAD_P/N, LANE0..3_TXD_P/N, LANE0..3_RXD_P/N   pads
#
# The parameter names follow Microchip's PolarFire HDMI 2.0 display component
# (github.com/MicrochipTech/fpga-hls-libraries, vision/rtl/display_sd_component/
# HDMI_2p0, Apache-2.0: same lane order, TMDS clock on lane 0), with the lane
# rate set for 1920x1080p60.  This script has not been run in Libero here:
# open PF_XCVR_HDMI in the configurator afterwards and check that it reports
# TX data rate 1485 Mbps, TX PCS-fabric width 40 and a TX clock of 37.125 MHz,
# and that PF_TX_PLL_HDMI matches the clock the configurator asks for.
# Core versions can be overridden before sourcing (set PF_XCVR_ERM_version ...).
# ============================================================================

if {![info exists HDMI_LANE_RATE]} { set HDMI_LANE_RATE 1485 }     ;# Mbps: 742.5 for 720p60
if {![info exists PF_XCVR_REF_CLK_version]} { set PF_XCVR_REF_CLK_version 1.0.103 }
if {![info exists PF_TX_PLL_version]} { set PF_TX_PLL_version 2.0.304 }
if {![info exists PF_XCVR_ERM_version]} { set PF_XCVR_ERM_version 3.1.206 }

create_and_configure_core -core_vlnv "Actel:SgCore:PF_XCVR_REF_CLK:${PF_XCVR_REF_CLK_version}" \
    -component_name {PF_XCVR_REF_CLK_HDMI} -params {\
    "ENABLE_FAB_CLK_0:false" \
    "ENABLE_FAB_CLK_1:false" \
    "ENABLE_REF_CLK_0:true" \
    "ENABLE_REF_CLK_1:false" \
    "REF_CLK_MODE_0:DIFFERENTIAL" \
    "REF_CLK_MODE_1:LVCMOS" }

create_and_configure_core -core_vlnv "Actel:SgCore:PF_TX_PLL:${PF_TX_PLL_version}" \
    -component_name {PF_TX_PLL_HDMI} -params [list \
    "TxPLL_REF:148.5" \
    "TxPLL_SOURCE:DEDICATED" \
    "TxPLL_JITTER_MODE_REFCLK_SEL:DEDICATED" \
    "TxPLL_OUT:${HDMI_LANE_RATE}" \
    "TxPLL_BANDWIDTH:Low" \
    "TxPLL_MODE:NORMAL" \
    "TxPLL_INTEGER_MODE:false" ]

create_and_configure_core -core_vlnv "Actel:SystemBuilder:PF_XCVR_ERM:${PF_XCVR_ERM_version}" \
    -component_name {PF_XCVR_HDMI} -params [list \
    "UI_ENABLE_PMA_MODE:true" \
    "UI_ENABLE_8B10B_MODE:false" \
    "UI_ENABLE_64B66B:false" \
    "UI_NUMBER_OF_LANES:4" \
    "UI_TX_RX_MODE:Tx and Rx (Independent)" \
    "UI_TX_DATA_RATE:${HDMI_LANE_RATE}" \
    "UI_TX_PCS_FAB_IF_WIDTH:40" \
    "UI_RX_DATA_RATE:${HDMI_LANE_RATE}" \
    "UI_RX_PCS_FAB_IF_WIDTH:10" \
    "UI_CDR_REFERENCE_CLK_FREQ:148.50" \
    "UI_CDR_REFERENCE_CLK_SOURCE:Dedicated" \
    "UI_CDR_LOCK_MODE:Lock to data" \
    "UI_INTERFACE_TXCLOCK:Regional" \
    "UI_INTERFACE_RXCLOCK:Regional" \
    "UI_PMA_ARST_N:TX and RX PMA" \
    "UI_PCS_ARST_N:RX Only" ]

# ---------------------------------------------------------------- SmartDesign
set sd HDMI_PHY
create_smartdesign -sd_name $sd
sd_instantiate_component -sd_name $sd -component_name {PF_XCVR_REF_CLK_HDMI} -instance_name {REFCLK}
sd_instantiate_component -sd_name $sd -component_name {PF_TX_PLL_HDMI} -instance_name {TXPLL}
sd_instantiate_component -sd_name $sd -component_name {PF_XCVR_HDMI} -instance_name {XCVR}

sd_create_scalar_port -sd_name $sd -port_name {RESET_N} -port_direction {IN}
sd_create_scalar_port -sd_name $sd -port_name {TX_CLK} -port_direction {OUT}
sd_create_scalar_port -sd_name $sd -port_name {TX_CLK_STABLE} -port_direction {OUT}
foreach l {0 1 2 3} {
    sd_create_bus_port -sd_name $sd -port_name "LANE${l}_TX_DATA" -port_direction {IN} -port_range {[39:0]}
    sd_connect_pins -sd_name $sd -pin_names [list "LANE${l}_TX_DATA" "XCVR:LANE${l}_TX_DATA"]
    sd_connect_pins -sd_name $sd -pin_names [list "RESET_N" "XCVR:LANE${l}_PCS_ARST_N" "XCVR:LANE${l}_PMA_ARST_N"]
    sd_connect_pins -sd_name $sd -pin_names [list "TXPLL:REF_CLK" "XCVR:LANE${l}_CDR_REF_CLK_0"]
    sd_connect_pins_to_constant -sd_name $sd -pin_names "XCVR:LANE${l}_RX_SLIP" -value {GND}
    foreach p {RX_CLK_R RX_IDLE RX_READY RX_VAL RX_BYPASS_DATA RX_DATA} {
        catch { sd_mark_pins_unused -sd_name $sd -pin_names "XCVR:LANE${l}_${p}" }
    }
    if {$l != 0} {
        catch { sd_mark_pins_unused -sd_name $sd -pin_names "XCVR:LANE${l}_TX_CLK_R" }
        catch { sd_mark_pins_unused -sd_name $sd -pin_names "XCVR:LANE${l}_TX_CLK_STABLE" }
    }
}
sd_connect_pins -sd_name $sd -pin_names {"REFCLK:REF_CLK" "TXPLL:REF_CLK"}
sd_connect_pins -sd_name $sd -pin_names {"TXPLL:CLKS_TO_XCVR" "XCVR:CLKS_FROM_TXPLL_0"}
sd_connect_pins -sd_name $sd -pin_names {"XCVR:LANE0_TX_CLK_R" "TX_CLK"}
sd_connect_pins -sd_name $sd -pin_names {"XCVR:LANE0_TX_CLK_STABLE" "TX_CLK_STABLE"}
catch { sd_mark_pins_unused -sd_name $sd -pin_names {TXPLL:PLL_LOCK} }
auto_promote_pad_pins -promote_all 1
save_smartdesign -sd_name $sd
generate_component -component_name $sd
