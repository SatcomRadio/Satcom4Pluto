#!/bin/sh
# Adds the "satcom4pluto" device type to OpenWebRX / OpenWebRX+ (SoapySDR module, device type,
# feature registration), then restarts OpenWebRX.
#   sudo sh install_openwebrx.sh            finds the running OpenWebRX docker container
#   sudo sh install_openwebrx.sh NAME       docker container NAME
#   sudo sh install_openwebrx.sh --native   OpenWebRX installed directly on this PC (package)
# Files go INTO the container: re-run after the container is re-created (satcom4pluto does
# that by itself when OWRX_CONTAINER is set in /etc/satcom4pluto.conf).
set -e
cd "$(dirname "$0")"
T=${1:-}
if [ "$T" = "--native" ]; then
  X=""
else
  command -v docker > /dev/null || { echo "docker not found (OpenWebRX installed natively? use --native)"; exit 1; }
  if [ -z "$T" ]; then
    T=$(docker ps --format '{{.Names}} {{.Image}}' | awk 'tolower($0) ~ /openwebrx/ {print $1; exit}')
    [ -n "$T" ] || { echo "no running OpenWebRX container found -- give its name: sudo sh install_openwebrx.sh NAME"; exit 1; }
  fi
  X="docker exec $T"
  echo "OpenWebRX container: $T"
fi
run() { $X sh -c "$1"; }
put() { if [ -z "$X" ]; then install -m 644 "$1" "$2"; else docker cp "$1" "$T:$2"; fi; }

arch=$(run "uname -m")
[ "$arch" = x86_64 ] || { echo "ERROR: this satcom4pluto module is built for x86_64, OpenWebRX runs on $arch"; exit 1; }
abi=$(run "SoapySDRUtil --info 2> /dev/null | sed -n 's/.*ABI Version: v//p'" || true)
if [ -n "$abi" ] && [ "$abi" != "0.8" ]; then echo "ERROR: OpenWebRX uses SoapySDR ABI $abi, the module needs 0.8"; exit 1; fi
src=$(run "find / -path '*/owrx/source/__init__.py' -not -path '/proc/*' 2> /dev/null | head -1")
[ -n "$src" ] || { echo "ERROR: OpenWebRX (owrx/source) not found"; exit 1; }
src=$(dirname "$src")
feat=$(dirname "$src")/feature.py

run "mkdir -p /usr/local/lib/SoapySDR/modules0.8"
put libsatcom4plutoSupport.so /usr/local/lib/SoapySDR/modules0.8/libsatcom4plutoSupport.so
put satcom4pluto.py "$src/satcom4pluto.py"
if ! run "grep -q 'satcom4pluto begin' '$feat'"; then
  put feature_patch.py /tmp/satcom4pluto_feature_patch.py
  run "cat /tmp/satcom4pluto_feature_patch.py >> '$feat' && rm -f /tmp/satcom4pluto_feature_patch.py"
fi
if run "soapy_connector --listdrivers" | grep -qx satcom4pluto; then
  echo "SoapySDR module satcom4pluto: OK"
else
  echo "ERROR: soapy_connector does not list the satcom4pluto module (library missing in OpenWebRX?):"
  run "SoapySDRUtil --info 2>&1 | tail -5" || true
  exit 1
fi
echo "device type: $src/satcom4pluto.py, feature registered in $feat"
if [ -z "$X" ]; then
  systemctl restart openwebrx 2> /dev/null && echo "OpenWebRX restarted" || echo "restart OpenWebRX now"
else
  docker restart "$T" > /dev/null && echo "container $T restarted"
  if [ -f /etc/satcom4pluto.conf ] && ! grep -q "^OWRX_CONTAINER=$T\$" /etc/satcom4pluto.conf; then
    sed -i "s/^OWRX_CONTAINER=.*/OWRX_CONTAINER=$T/" /etc/satcom4pluto.conf
    systemctl restart satcom4pluto 2> /dev/null || true
  fi
fi
echo "OpenWebRX: Settings -> SDR devices and profiles -> Add device -> \"satcom4pluto (Pluto satellite-downlink channelizer)\""
