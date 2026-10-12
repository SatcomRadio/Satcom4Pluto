#!/usr/bin/env python3
"""Xilinx .bit -> .bin for the Linux fpga_manager (header stripped, 32-bit words byte-swapped).
usage: bit2bin.py in.bit out.bin"""
import sys, struct
d = open(sys.argv[1], "rb").read()
i = 2 + struct.unpack(">H", d[:2])[0]          # field 1
i += 2 + struct.unpack(">H", d[i:i + 2])[0] if False else 0
# walk the keyed fields a..e
i = 2 + struct.unpack(">H", d[0:2])[0]
i += 2                                           # 0x0001
while True:
    k = d[i]; i += 1
    if k in b"abcd":
        n = struct.unpack(">H", d[i:i + 2])[0]; i += 2 + n
    elif k == ord("e"):
        n = struct.unpack(">I", d[i:i + 4])[0]; i += 4
        payload = d[i:i + n]
        break
    else:
        raise SystemExit("bad .bit header")
payload += b"\0" * (-len(payload) % 4)
out = bytearray(len(payload))
out[0::4], out[1::4], out[2::4], out[3::4] = payload[3::4], payload[2::4], payload[1::4], payload[0::4]
open(sys.argv[2], "wb").write(out)
print("%s: %d bytes" % (sys.argv[2], len(out)))
