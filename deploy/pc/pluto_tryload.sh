#!/bin/sh
# SPDX-License-Identifier: MIT
# Try the channelizer bitstream WITHOUT flashing: load it into the FPGA of the
# running Pluto (lost at power-off / reboot). Run on the Pluto:
#   scp -O firmware/sgch_pl.bin tools/pluto_tryload.sh root@192.168.5.5:/tmp/
#   ssh root@192.168.5.5 sh /tmp/pluto_tryload.sh
# Reloading the PL resets registers the drivers set at boot; they are restored
# below. (FPGA IQ correction stays off until the next boot.)
set -e
BIN=${1:-/tmp/sgch_pl.bin}
killall sgchd 2>/dev/null || true
cp "$BIN" /lib/firmware/sgch_pl.bin
echo 0 > /sys/class/fpga_manager/fpga0/flags
echo sgch_pl.bin > /sys/class/fpga_manager/fpga0/firmware
devmem 0x79020040 32 0x3          # axi_ad9361 RX core out of reset
devmem 0x79024040 32 0x3          # TX core
devmem 0x7C400080 32 0            # RX DMA: unmask interrupts
devmem 0x7C420080 32 0            # TX DMA
for c in 0 1 2 3; do devmem $((0x79020400 + c * 0x40)) 32 0x51; done   # enable, data format, sign extension
echo 1 > /sys/kernel/debug/iio/iio:device0/initialize                    # re-tune the LVDS interface
echo "channelizer id: $(devmem 0x43C00000) (0x53474348 = SGCH)"
