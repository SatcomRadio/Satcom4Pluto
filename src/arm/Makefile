# SPDX-License-Identifier: MIT
# Pluto ARM daemon (static, runs on the stock Pluto firmware) and the host test harness.
#   make            -> sgchd        (needs arm-linux-gnueabihf-gcc, e.g. apt install gcc-arm-linux-gnueabihf)
#   make det2_test  -> host build of the detector test harness (src/reference/cmp_c.py uses it)
CROSS   ?= arm-linux-gnueabihf-
CFLAGS  ?= -O2 -Wall
ARMFLAGS = -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard

all: sgchd

sgchd: sgchd.c det2.c det2.h
	$(CROSS)gcc $(CFLAGS) $(ARMFLAGS) -static -o $@ sgchd.c det2.c -lm
	$(CROSS)strip $@

det2_test: det2_test.c det2.c det2.h
	gcc $(CFLAGS) -o $@ det2_test.c det2.c -lm

clean:
	rm -f sgchd det2_test
.PHONY: all clean
