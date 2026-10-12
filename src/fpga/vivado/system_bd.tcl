# Derived from the Analog Devices HDL repository (projects/pluto, hdl_2026_r1):
# distributed under the license terms of https://github.com/analogdevicesinc/hdl (see its LICENSE).
#
# Block design: ADALM-Pluto firmware address map on the 7Z010-CLG400 LVDS clone,
# plus the satellite-downlink channelizer (sgc_top) between the RX decimator
# and util_cpack2. Based on ADI hdl_2026_r1 projects/pluto/system_bd.tcl
# (no TDD core, no PL SPI, DMA without sync start: matches the v0.38 firmware).

source $ad_hdl_dir/projects/common/xilinx/adi_fir_filter_bd.tcl
global sgc_dir
if {![info exists sgc_tx_fir]} { set sgc_tx_fir 0 }

# default ports
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddrx_rtl:1.0 ddr
create_bd_intf_port -mode Master -vlnv xilinx.com:display_processing_system7:fixedio_rtl:1.0 fixed_io

create_bd_port -dir O spi0_csn_2_o
create_bd_port -dir O spi0_csn_1_o
create_bd_port -dir O spi0_csn_0_o
create_bd_port -dir I spi0_csn_i
create_bd_port -dir I spi0_clk_i
create_bd_port -dir O spi0_clk_o
create_bd_port -dir I spi0_sdo_i
create_bd_port -dir O spi0_sdo_o
create_bd_port -dir I spi0_sdi_i

create_bd_port -dir I -from 16 -to 0 gpio_i
create_bd_port -dir O -from 16 -to 0 gpio_o
create_bd_port -dir O -from 16 -to 0 gpio_t

# processing system (only the PL-facing settings matter: the board's FSBL
# configures DDR, MIO and clocks)
ad_ip_instance processing_system7 sys_ps7
ad_ip_parameter sys_ps7 CONFIG.PCW_PACKAGE_NAME clg400
ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 3.3V}
ad_ip_parameter sys_ps7 CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V}
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP1 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP2 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_CLK1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_EN_RST1_PORT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ 100.0
ad_ip_parameter sys_ps7 CONFIG.PCW_FPGA1_PERIPHERAL_FREQMHZ 200.0
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_EMIO_GPIO_IO 17
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI1_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_UART1_UART1_IO {MIO 8 .. 9}
ad_ip_parameter sys_ps7 CONFIG.PCW_I2C1_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SD0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SD0_SD0_IO {MIO 40 .. 45}
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27}
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_GRP_MDIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53}
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_SPI0_SPI0_IO EMIO
ad_ip_parameter sys_ps7 CONFIG.PCW_TTC0_PERIPHERAL_ENABLE 0
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_FABRIC_INTERRUPT 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_PERIPHERAL_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_USB0_IO {MIO 28 .. 39}
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_USB0_RESET_IO {MIO 46}
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_ENABLE 1
ad_ip_parameter sys_ps7 CONFIG.PCW_GPIO_MIO_GPIO_IO MIO
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_INTR 1
ad_ip_parameter sys_ps7 CONFIG.PCW_IRQ_F2P_MODE REVERSE
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_PARTNO {MT41K256M16 RE-125}
ad_ip_parameter sys_ps7 CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {32 Bit}

ad_ip_instance ilconcat sys_concat_intc
ad_ip_parameter sys_concat_intc CONFIG.NUM_PORTS 16

ad_ip_instance proc_sys_reset sys_rstgen
ad_ip_parameter sys_rstgen CONFIG.C_EXT_RST_WIDTH 1

ad_connect  sys_cpu_clk sys_ps7/FCLK_CLK0
ad_connect  sys_200m_clk sys_ps7/FCLK_CLK1
ad_connect  sys_cpu_reset sys_rstgen/peripheral_reset
ad_connect  sys_cpu_resetn sys_rstgen/peripheral_aresetn
ad_connect  sys_cpu_clk sys_rstgen/slowest_sync_clk
ad_connect  sys_rstgen/ext_reset_in sys_ps7/FCLK_RESET0_N

ad_connect  ddr sys_ps7/DDR
ad_connect  gpio_i sys_ps7/GPIO_I
ad_connect  gpio_o sys_ps7/GPIO_O
ad_connect  gpio_t sys_ps7/GPIO_T
ad_connect  fixed_io sys_ps7/FIXED_IO

ad_connect  spi0_csn_2_o sys_ps7/SPI0_SS2_O
ad_connect  spi0_csn_1_o sys_ps7/SPI0_SS1_O
ad_connect  spi0_csn_0_o sys_ps7/SPI0_SS_O
ad_connect  spi0_csn_i sys_ps7/SPI0_SS_I
ad_connect  spi0_clk_i sys_ps7/SPI0_SCLK_I
ad_connect  spi0_clk_o sys_ps7/SPI0_SCLK_O
ad_connect  spi0_sdo_i sys_ps7/SPI0_MOSI_I
ad_connect  spi0_sdo_o sys_ps7/SPI0_MOSI_O
ad_connect  spi0_sdi_i sys_ps7/SPI0_MISO_I

ad_connect  sys_concat_intc/dout sys_ps7/IRQ_F2P
for {set i 0} {$i < 16} {incr i} {
  if {$i != 12 && $i != 13 && $i != 15} { ad_connect sys_concat_intc/In$i GND }
}

# PL I2C at 0x41600000 is in the firmware device tree but has no pins on
# this board: a stub answers there (status = idle, FIFOs empty)
add_files -norecurse $sgc_dir/rtl/sgc_axi_stub.v
update_compile_order -fileset sources_1
create_bd_cell -type module -reference sgc_axi_stub iic_stub
ad_cpu_interconnect 0x41600000 iic_stub
ad_connect  sys_concat_intc/In15 iic_stub/irq

# ad9361 (LVDS)
create_bd_port -dir I rx_clk_in_p
create_bd_port -dir I rx_clk_in_n
create_bd_port -dir I rx_frame_in_p
create_bd_port -dir I rx_frame_in_n
create_bd_port -dir I -from 5 -to 0 rx_data_in_p
create_bd_port -dir I -from 5 -to 0 rx_data_in_n
create_bd_port -dir O tx_clk_out_p
create_bd_port -dir O tx_clk_out_n
create_bd_port -dir O tx_frame_out_p
create_bd_port -dir O tx_frame_out_n
create_bd_port -dir O -from 5 -to 0 tx_data_out_p
create_bd_port -dir O -from 5 -to 0 tx_data_out_n
create_bd_port -dir O enable
create_bd_port -dir O txnrx
create_bd_port -dir I up_enable
create_bd_port -dir I up_txnrx

ad_ip_instance axi_ad9361 axi_ad9361
ad_ip_parameter axi_ad9361 CONFIG.ID 0
ad_ip_parameter axi_ad9361 CONFIG.CMOS_OR_LVDS_N 0
ad_ip_parameter axi_ad9361 CONFIG.MODE_1R1T 0
ad_ip_parameter axi_ad9361 CONFIG.ADC_INIT_DELAY 21
# trimmed to fit the 7Z010 next to the channelizer (no TX DDS / TX IQ correction / TDD)
ad_ip_parameter axi_ad9361 CONFIG.TDD_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.DAC_DDS_DISABLE 1
ad_ip_parameter axi_ad9361 CONFIG.DAC_IQCORRECTION_DISABLE 1

ad_ip_instance axi_dmac axi_ad9361_dac_dma
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_TYPE_SRC 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_TYPE_DEST 1
ad_ip_parameter axi_ad9361_dac_dma CONFIG.CYCLIC 1
ad_ip_parameter axi_ad9361_dac_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_ad9361_dac_dma CONFIG.DMA_DATA_WIDTH_DEST 64

ad_ip_instance util_upack2 tx_upack

ad_ip_instance axi_dmac axi_ad9361_adc_dma
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_TYPE_SRC 2
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_TYPE_DEST 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.CYCLIC 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.SYNC_TRANSFER_START 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.AXI_SLICE_SRC 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.AXI_SLICE_DEST 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_2D_TRANSFER 0
ad_ip_parameter axi_ad9361_adc_dma CONFIG.DMA_DATA_WIDTH_SRC 64

ad_add_decimation_filter "rx_fir_decimator" 8 2 1 {61.44} {61.44} \
                         "$ad_hdl_dir/library/util_fir_int/coefile_int.coe"
ad_ip_instance ilslice decim_slice
ad_ip_instance util_cpack2 cpack

# channelizer
add_files -norecurse [list \
  $sgc_dir/rtl/sgc_top.v \
  $sgc_dir/rtl/sgc_core.v \
  $sgc_dir/rtl/sgc_fft.v \
  $sgc_dir/rtl/sgc_afifo.v]
add_files -norecurse -fileset constrs_1 $sgc_dir/vivado/sgc_constr.xdc
update_compile_order -fileset sources_1
create_bd_cell -type module -reference sgc_top sgc

ad_connect  rx_clk_in_p axi_ad9361/rx_clk_in_p
ad_connect  rx_clk_in_n axi_ad9361/rx_clk_in_n
ad_connect  rx_frame_in_p axi_ad9361/rx_frame_in_p
ad_connect  rx_frame_in_n axi_ad9361/rx_frame_in_n
ad_connect  rx_data_in_p axi_ad9361/rx_data_in_p
ad_connect  rx_data_in_n axi_ad9361/rx_data_in_n
ad_connect  tx_clk_out_p axi_ad9361/tx_clk_out_p
ad_connect  tx_clk_out_n axi_ad9361/tx_clk_out_n
ad_connect  tx_frame_out_p axi_ad9361/tx_frame_out_p
ad_connect  tx_frame_out_n axi_ad9361/tx_frame_out_n
ad_connect  tx_data_out_p axi_ad9361/tx_data_out_p
ad_connect  tx_data_out_n axi_ad9361/tx_data_out_n
ad_connect  enable axi_ad9361/enable
ad_connect  txnrx axi_ad9361/txnrx
ad_connect  up_enable axi_ad9361/up_enable
ad_connect  up_txnrx axi_ad9361/up_txnrx

ad_connect  axi_ad9361/tdd_sync GND
ad_connect  sys_200m_clk axi_ad9361/delay_clk
ad_connect  axi_ad9361/l_clk axi_ad9361/clk

# RX: ad9361 -> decimator -> sgc -> cpack -> DMA
ad_connect axi_ad9361/l_clk rx_fir_decimator/aclk
ad_connect axi_ad9361/adc_valid_i0 rx_fir_decimator/valid_in_0
ad_connect axi_ad9361/adc_enable_i0 rx_fir_decimator/enable_in_0
ad_connect axi_ad9361/adc_data_i0 rx_fir_decimator/data_in_0
ad_connect axi_ad9361/adc_valid_q0 rx_fir_decimator/valid_in_1
ad_connect axi_ad9361/adc_enable_q0 rx_fir_decimator/enable_in_1
ad_connect axi_ad9361/adc_data_q0 rx_fir_decimator/data_in_1
ad_connect axi_ad9361/up_adc_gpio_out decim_slice/Din
ad_connect rx_fir_decimator/active decim_slice/Dout

ad_connect axi_ad9361/l_clk sgc/adc_clk
ad_connect axi_ad9361/rst sgc/adc_rst
ad_connect rx_fir_decimator/valid_out_0 sgc/in_valid_0
ad_connect rx_fir_decimator/enable_out_0 sgc/in_enable_0
ad_connect rx_fir_decimator/data_out_0 sgc/in_data_0
ad_connect rx_fir_decimator/valid_out_1 sgc/in_valid_1
ad_connect rx_fir_decimator/enable_out_1 sgc/in_enable_1
ad_connect rx_fir_decimator/data_out_1 sgc/in_data_1
ad_connect axi_ad9361/adc_enable_i1 sgc/in_enable_2
ad_connect axi_ad9361/adc_data_i1 sgc/in_data_2
ad_connect axi_ad9361/adc_enable_q1 sgc/in_enable_3
ad_connect axi_ad9361/adc_data_q1 sgc/in_data_3

ad_connect axi_ad9361/l_clk cpack/clk
ad_connect axi_ad9361/rst cpack/reset
ad_connect sgc/out_valid cpack/fifo_wr_en
ad_connect sgc/out_enable_0 cpack/enable_0
ad_connect sgc/out_data_0 cpack/fifo_wr_data_0
ad_connect sgc/out_enable_1 cpack/enable_1
ad_connect sgc/out_data_1 cpack/fifo_wr_data_1
ad_connect sgc/out_enable_2 cpack/enable_2
ad_connect sgc/out_data_2 cpack/fifo_wr_data_2
ad_connect sgc/out_enable_3 cpack/enable_3
ad_connect sgc/out_data_3 cpack/fifo_wr_data_3
ad_connect axi_ad9361_adc_dma/fifo_wr cpack/packed_fifo_wr
ad_connect cpack/fifo_wr_overflow axi_ad9361/adc_dovf
ad_connect axi_ad9361/l_clk axi_ad9361_adc_dma/fifo_wr_clk

# TX
ad_connect axi_ad9361/l_clk tx_upack/clk
ad_connect axi_ad9361/rst tx_upack/reset
ad_connect axi_ad9361/dac_enable_i1 tx_upack/enable_2
ad_connect axi_ad9361/dac_data_i1 tx_upack/fifo_rd_data_2
ad_connect axi_ad9361/dac_enable_q1 tx_upack/enable_3
ad_connect axi_ad9361/dac_data_q1 tx_upack/fifo_rd_data_3
ad_connect tx_upack/s_axis axi_ad9361_dac_dma/m_axis
ad_connect tx_upack/fifo_rd_underflow axi_ad9361/dac_dunf
ad_connect axi_ad9361/l_clk axi_ad9361_dac_dma/m_axis_aclk

if {$sgc_tx_fir} {
  ad_add_interpolation_filter "tx_fir_interpolator" 8 2 1 {61.44} {7.68} \
                               "$ad_hdl_dir/library/util_fir_int/coefile_int.coe"
  ad_ip_instance ilslice interp_slice
  ad_connect axi_ad9361/l_clk tx_fir_interpolator/aclk
  ad_connect axi_ad9361/dac_enable_i0 tx_fir_interpolator/dac_enable_0
  ad_connect axi_ad9361/dac_valid_i0 tx_fir_interpolator/dac_valid_0
  ad_connect axi_ad9361/dac_data_i0 tx_fir_interpolator/data_out_0
  ad_connect axi_ad9361/dac_enable_q0 tx_fir_interpolator/dac_enable_1
  ad_connect axi_ad9361/dac_valid_q0 tx_fir_interpolator/dac_valid_1
  ad_connect axi_ad9361/dac_data_q0 tx_fir_interpolator/data_out_1
  ad_connect tx_upack/fifo_rd_data_0 tx_fir_interpolator/data_in_0
  ad_connect tx_upack/enable_0 tx_fir_interpolator/enable_out_0
  ad_connect tx_upack/fifo_rd_data_1 tx_fir_interpolator/data_in_1
  ad_connect tx_upack/enable_1 tx_fir_interpolator/enable_out_1
  ad_ip_instance ilvector_logic logic_or [list C_OPERATION {or} C_SIZE 1]
  ad_connect logic_or/Op1 tx_fir_interpolator/valid_out_0
  ad_connect logic_or/Op2 axi_ad9361/dac_valid_i1
  ad_connect logic_or/Res tx_upack/fifo_rd_en
  ad_connect axi_ad9361/up_dac_gpio_out interp_slice/Din
  ad_connect tx_fir_interpolator/active interp_slice/Dout
} else {
  # no TX interpolator: TX works at the AD9361 rate only (>= 2.08 MS/s)
  ad_connect tx_upack/fifo_rd_data_0 axi_ad9361/dac_data_i0
  ad_connect tx_upack/enable_0 axi_ad9361/dac_enable_i0
  ad_connect tx_upack/fifo_rd_data_1 axi_ad9361/dac_data_q0
  ad_connect tx_upack/enable_1 axi_ad9361/dac_enable_q0
  ad_ip_instance ilvector_logic logic_or [list C_OPERATION {or} C_SIZE 1]
  ad_connect logic_or/Op1 axi_ad9361/dac_valid_i0
  ad_connect logic_or/Op2 axi_ad9361/dac_valid_i1
  ad_connect logic_or/Res tx_upack/fifo_rd_en
}

# interconnects (same addresses as the Pluto firmware) + channelizer
ad_cpu_interconnect 0x79020000 axi_ad9361
ad_cpu_interconnect 0x7C400000 axi_ad9361_adc_dma
ad_cpu_interconnect 0x7C420000 axi_ad9361_dac_dma
ad_cpu_interconnect 0x43C00000 sgc
set_property range 64K [get_bd_addr_segs -of_objects [get_bd_addr_spaces sys_ps7/Data] -filter {NAME =~ *sgc*}]

ad_connect sys_cpu_clk sys_ps7/S_AXI_HP1_ACLK
ad_connect axi_ad9361_adc_dma/m_dest_axi sys_ps7/S_AXI_HP1
create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_ad9361_adc_dma/m_dest_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP1/HP1_DDR_LOWOCM] \
                    SEG_sys_ps7_HP1_DDR_LOWOCM
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP2_ACLK
ad_connect axi_ad9361_dac_dma/m_src_axi sys_ps7/S_AXI_HP2
create_bd_addr_seg -range 0x40000000 -offset 0x00000000 \
                    [get_bd_addr_spaces axi_ad9361_dac_dma/m_src_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP2/HP2_DDR_LOWOCM] \
                    SEG_sys_ps7_HP2_DDR_LOWOCM
ad_connect sys_cpu_clk axi_ad9361_dac_dma/m_src_axi_aclk
ad_connect sys_cpu_clk axi_ad9361_adc_dma/m_dest_axi_aclk
ad_connect sys_cpu_resetn axi_ad9361_adc_dma/m_dest_axi_aresetn
ad_connect sys_cpu_resetn axi_ad9361_dac_dma/m_src_axi_aresetn

# interrupts (as on the Pluto: rx dma 13, tx dma 12)
ad_connect sys_concat_intc/In13 axi_ad9361_adc_dma/irq
ad_connect sys_concat_intc/In12 axi_ad9361_dac_dma/irq
