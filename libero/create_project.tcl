# ============================================================================
# Libero SoC: stand-alone project for the H.264 decoder core
#
#   libero SCRIPT:create_project.tcl SCRIPT_ARGS:"<project_dir>"
#
# Creates a PolarFire SoC MPFS250T project, imports the VHDL-2008 sources,
# sets h264_dec_top as root and runs synthesis (Synplify Pro) so that the
# resource and timing estimates of the core can be checked.  For the Video
# Kit design add the core to your SmartDesign instead (see README, section 7).
#
# Check die / package / speed grade against the device on your kit
# (Libero: Project > Project Settings) - adjust the variables below.
# ============================================================================
set proj_dir [lindex $argv 0]
if {$proj_dir == ""} { set proj_dir "./h264_dec_prj" }
set src_dir [file normalize [file join [file dirname [info script]] .. rtl]]

set die     "MPFS250T"
set package "FCG1152"
set speed   "-1"
set voltage "1.0"
set range   "EXT"

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

run_tool -name {SYNTHESIZE}
save_project
