#!/bin/sh
# sudo sh uninstall_linux.sh [OpenWebRX container name | --native]
# Removes the loader service and the satcom4pluto files from OpenWebRX; power-cycle the Pluto afterwards.
systemctl disable --now satcom4pluto 2> /dev/null
rm -f /etc/systemd/system/satcom4pluto.service
systemctl daemon-reload
T=${1:-$(sed -n 's/^OWRX_CONTAINER=//p' /etc/satcom4pluto.conf 2> /dev/null)}
if [ "$T" = "--native" ]; then X=""; elif [ -n "$T" ]; then X="docker exec $T"; else X=""; fi
$X sh -c 'rm -f /usr/local/lib/SoapySDR/modules0.8/libsatcom4plutoSupport.so;
  s=$(find / -path "*/owrx/source/satcom4pluto.py" -not -path "/proc/*" 2> /dev/null | head -1); [ -n "$s" ] && rm -f "$s";
  f=$(find / -path "*/owrx/feature.py" -not -path "/proc/*" 2> /dev/null | head -1);
  [ -n "$f" ] && sed -i "/# --- satcom4pluto begin/,/# --- satcom4pluto end ---/d" "$f"' || true
[ -n "$X" ] && docker restart "$T" > /dev/null
rm -rf /usr/local/share/satcom4pluto
echo "removed (settings kept in /etc/satcom4pluto.conf). Remove the satcom4pluto device in OpenWebRX's settings too."
