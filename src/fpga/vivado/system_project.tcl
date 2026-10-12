# Derived from the Analog Devices HDL repository (projects/pluto, hdl_2026_r1):
# distributed under the license terms of https://github.com/analogdevicesinc/hdl (see its LICENSE).
# Build: vivado -mode batch -source system_project.tcl -tclargs <hdl dir> <sgch dir> [tx_fir 0|1]
# (hdl dir = ADI hdl repository, branch hdl_2026_r1, libraries built by this script)

set ad_hdl_dir [file normalize [lindex $argv 0]]
set sgc_dir    [file normalize [lindex $argv 1]]
if {[llength $argv] > 2} { set sgc_tx_fir [lindex $argv 2] }
set ::env(ADI_IGNORE_VERSION_CHECK) 1
set ADI_IGNORE_VERSION_CHECK 1
set IGNORE_VERSION_CHECK 1
source $ad_hdl_dir/scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl

set p_device "xc7z010clg400-1"
adi_project_create sgch_pluto 0 {} $p_device

adi_project_files sgch_pluto [list \
  "$sgc_dir/vivado/system_top.v" \
  "$sgc_dir/vivado/system_constr.xdc" \
  "$ad_hdl_dir/library/common/ad_iobuf.v"]

set rc [catch {adi_project_run sgch_pluto} err]
# reports for checking the fit (written also when implementation fails)
catch {
  if {[get_property STATUS [get_runs impl_1]] eq "write_bitstream Complete!"} {
    open_run impl_1
  } else {
    open_run synth_1
  }
  report_utilization -hierarchical -hierarchical_depth 4 -file $sgc_dir/vivado/util_hier.rpt
  report_utilization -file $sgc_dir/vivado/util.rpt
  report_timing_summary -file $sgc_dir/vivado/timing.rpt
}
if {$rc} { error $err }
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
