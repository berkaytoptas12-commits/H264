# ============================================================================
# Libero SoC: stand-alone project for the H.264 decoder core
#
#   libero SCRIPT:create_project.tcl SCRIPT_ARGS:"<project_dir>"
#
# Creates a PolarFire SoC MPFS250TS project, imports the VHDL-2008 sources,
# sets h264_dec_top as root, attaches h264_dec.sdc (100 MHz) and runs
# synthesis (Synplify Pro) so that the resource and timing estimates of the
# core can be checked.  For the Video Kit design add the core to your
# SmartDesign instead (see README, section 7).
#
# The MPFS250 Video Kit carries an MPFS250TS-1FCG1152I (die MPFS250TS,
# package FCG1152, speed -1, industrial range).  Check this against the
# marking on your board and adjust the variables below if needed.
# ============================================================================
set proj_dir [lindex $argv 0]
if {$proj_dir == ""} { set proj_dir "./h264_dec_prj" }
set src_dir [file normalize [file join [file dirname [info script]] .. rtl]]

set die     "MPFS250TS"
set package "FCG1152"
set speed   "-1"
set voltage "1.0"
set range   "IND"

new_project -location $proj_dir -name {h264_dec} -project_description {H.264 decoder core} \
    -block_mode 0 -standalone_peripheral_initialization 0 -instantiate_in_smartdesign 1 \
    -ondemand_build_dh 1 -use_relative_path 0 -linked_files_root_dir_env {} \
    -hdl {VHDL} -family {PolarFireSoC} -die $die -package $package -speed $speed \
    -die_voltage $voltage -part_range $range

# VHDL-2008 is required (process(all), external names are only used in the testbench)
project_settings -vhdl_mode {VHDL_2008}

set files {
    h264_pkg.vhd h264_tables_pkg.vhd h264_ram.vhd h264_nal.vhd h264_bitreader.vhd
    h264_hdr.vhd h264_mbparse.vhd h264_mc.vhd h264_recon.vhd h264_deblock.vhd h264_dec_top.vhd
}
foreach f $files {
    import_files -convert_EDN_to_HDL 0 -library {work} -hdl_source [file join $src_dir $f]
}
import_files -sdc [file normalize [file join [file dirname [info script]] h264_dec.sdc]]

build_design_hierarchy
set_root -module {h264_dec_top::work}
organize_tool_files -tool {SYNTHESIZE} -file [file join $proj_dir constraint h264_dec.sdc] \
    -module {h264_dec_top::work} -input_type {constraint}

organize_tool_files -tool {PLACEROUTE} -file [file join $proj_dir constraint h264_dec.sdc] \
    -module {h264_dec_top::work} -input_type {constraint}
organize_tool_files -tool {VERIFYTIMING} -file [file join $proj_dir constraint h264_dec.sdc] \
    -module {h264_dec_top::work} -input_type {constraint}

run_tool -name {SYNTHESIZE}
# Stand-alone place & route needs every top level port on a pin (about 420
# I/Os); set run_pnr to 1 to try it, otherwise check timing in the SmartDesign
# project of the kit where the AXI ports are connected internally.
set run_pnr 0
if {$run_pnr} {
    run_tool -name {PLACEROUTE}
    run_tool -name {VERIFYTIMING}
}
save_project
