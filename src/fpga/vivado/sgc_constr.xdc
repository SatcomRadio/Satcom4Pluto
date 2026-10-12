# SPDX-License-Identifier: MIT
# Clock-domain crossings inside sgc_top (core = FCLK0, adc = axi_ad9361 l_clk)

# 2-FF synchronisers and slow status counters
set_false_path -to [get_cells -hier -filter {NAME =~ *sgc*/ar1_reg}]
set_false_path -to [get_cells -hier -filter {NAME =~ *sgc*/cr1_reg}]
set_false_path -to [get_cells -hier -filter {NAME =~ *sgc*/m1_reg}]
set_false_path -to [get_cells -hier -filter {NAME =~ *sgc*/iovf1_reg[*]}]

# async FIFO gray pointers: keep the skew below one destination period
set_max_delay -datapath_only 5.0 -from [get_cells -hier -filter {NAME =~ *sgc*/wg_reg[*]}] -to [get_cells -hier -filter {NAME =~ *sgc*/wg_r1_reg[*]}]
set_max_delay -datapath_only 5.0 -from [get_cells -hier -filter {NAME =~ *sgc*/rg_reg[*]}] -to [get_cells -hier -filter {NAME =~ *sgc*/rg_w1_reg[*]}]
# FIFO memory -> read register (data is stable while the pointer says it is valid)
set_max_delay -datapath_only 8.0 -from [get_cells -hier -filter {NAME =~ *sgc*/ififo/mem_reg*}] -to [get_cells -hier -filter {NAME =~ *sgc*/ififo/rdata_reg[*]}]
