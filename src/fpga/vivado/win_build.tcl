# Helper used to build on Windows from a running Vivado Tcl shell:
#   source win_build.tcl ; sgc_write_bats C:/sgcb
# then run C:/sgcb/build_libs.bat and C:/sgcb/build_proj.bat
proc sgc_write_bats {root {vivado {C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat}}} {
  set libs {interfaces axi_dmac/interfaces util_cdc util_axis_fifo axi_dmac axi_ad9361 util_pack/util_cpack2 util_pack/util_upack2}
  set w [string map {/ \\} $root]
  set f [open $root/build_libs.bat w]
  puts $f "@echo off"
  puts $f "set ADI_IGNORE_VERSION_CHECK=1"
  foreach l $libs {
    set n [file tail $l]
    set s [expr {$n eq "interfaces" ? "interfaces_ip.tcl" : "${n}_ip.tcl"}]
    set d [string map {/ \\} $root/hdl/library/$l]
    set lg "$w\\lib_[string map {/ _} $l].log"
    puts $f "echo building $l"
    puts $f "cd /d $d"
    puts $f "call \"$vivado\" -mode batch -notrace -source $s -log $lg -nojournal > NUL 2>&1"
  }
  puts $f "echo LIBS_DONE > $w\\libs_done.txt"
  close $f
  set f [open $root/build_proj.bat w]
  puts $f "@echo off"
  puts $f "set ADI_IGNORE_VERSION_CHECK=1"
  puts $f "cd /d $w\\sgch\\vivado"
  puts $f "call \"$vivado\" -mode batch -notrace -source system_project.tcl -log $w\\proj.log -nojournal -tclargs $root/hdl $root/sgch %1 > NUL 2>&1"
  puts $f "echo PROJ_DONE > $w\\proj_done.txt"
  close $f
}
