#!/bin/bash
# satcom4pluto: keep the Pluto loaded (run by satcom4pluto.service).
# Every 10 s: is the detector daemon answering on the Pluto? If not (Pluto just powered up or
# re-plugged), load the satcom4pluto FPGA image + detector into its RAM (satcom4pluto-load.sh).
# Every 5 min: is the satcom4pluto device type still inside the OpenWebRX container? (lost when the
# container is re-created) -> reinstall it.
D=/usr/local/share/satcom4pluto
P=${PLUTO:-192.168.5.5}
n=0
while true; do
  if ! timeout 3 bash -c "exec 3<>/dev/tcp/$P/30470" 2> /dev/null; then
    "$D/satcom4pluto-load.sh" "$P"
  fi
  if [ -n "$OWRX_CONTAINER" ] && [ $((n % 30)) -eq 0 ] && command -v docker > /dev/null; then
    if docker inspect -f '{{.State.Running}}' "$OWRX_CONTAINER" 2> /dev/null | grep -q true &&
       ! docker exec "$OWRX_CONTAINER" test -f /usr/local/lib/SoapySDR/modules0.8/libsatcom4plutoSupport.so; then
      echo "satcom4pluto: OpenWebRX container $OWRX_CONTAINER lost the satcom4pluto files -- reinstalling"
      sh "$D/install_openwebrx.sh" "$OWRX_CONTAINER"
    fi
  fi
  n=$((n + 1))
  sleep 10
done
