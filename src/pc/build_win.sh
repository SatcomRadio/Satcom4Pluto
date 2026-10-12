#!/bin/sh
# Cross-compile a self-contained sgch_rx.exe (Windows x64) with mingw-w64.
set -e
cd "$(dirname "$0")"
x86_64-w64-mingw32-g++-posix -O3 -std=c++14 -static -static-libgcc -static-libstdc++ \
  -o sgch_rx.exe sgch_rx_main.cpp sgch_rx.cpp channels.cpp -lws2_32 -lpthread
x86_64-w64-mingw32-strip sgch_rx.exe
