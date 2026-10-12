#!/bin/sh
# satcom4pluto: make sure the Pluto runs the satcom4pluto FPGA image and the detector daemon.
# The Pluto's own firmware is never changed: after every power-up this copies the FPGA image
# and sgchd into the Pluto's RAM (/tmp) over ssh and starts them. Run by satcom4pluto.service
# before sgch_rx; does nothing if the Pluto is already loaded since its last boot.
#   satcom4pluto-load.sh [PLUTO_IP]
P=${1:-${PLUTO:-192.168.5.5}}
D=/usr/local/share/satcom4pluto
export PLUTO_PASSWORD=${PLUTO_PASSWORD:-analog}
OPTS="-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR -o ConnectTimeout=5"
OPTS="$OPTS -o PubkeyAuthentication=no -o PreferredAuthentications=password,keyboard-interactive"
if command -v sshpass >/dev/null 2>&1; then
  pssh() { sshpass -p "$PLUTO_PASSWORD" ssh $OPTS root@"$P" "$@"; }
else
  # OpenSSH >= 8.4 reads the password from SSH_ASKPASS without a terminal
  export SSH_ASKPASS="$D/askpass.sh" SSH_ASKPASS_REQUIRE=force DISPLAY="${DISPLAY:-:0}"
  pssh() { setsid -w ssh $OPTS root@"$P" "$@" < /dev/null; }
  pssh_in() { setsid -w ssh $OPTS root@"$P" "$@"; }
fi
command -v pssh_in >/dev/null 2>&1 || pssh_in() { pssh "$@"; }

if ! pssh true; then
  echo "satcom4pluto-load: Pluto $P not reachable (switched on? USB network up?)"; exit 1
fi
if pssh "[ -f /tmp/satcom4pluto_loaded ] && pidof sgchd > /dev/null"; then
  echo "satcom4pluto-load: Pluto already running satcom4pluto"; exit 0
fi
echo "satcom4pluto-load: loading satcom4pluto into the Pluto's RAM"
pssh "killall sgchd 2> /dev/null; rm -f /tmp/satcom4pluto_loaded; true"
for f in sgch_pl.bin pluto_tryload.sh sgchd; do
  pssh_in "cat > /tmp/$f.new && mv -f /tmp/$f.new /tmp/$f" < "$D/$f" || { echo "satcom4pluto-load: copy of $f failed"; exit 1; }
done
pssh "chmod +x /tmp/sgchd; sh /tmp/pluto_tryload.sh /tmp/sgch_pl.bin" | tail -1
pssh "[ \"\$(devmem 0x43C00000)\" = 0x53474348 ] && touch /tmp/satcom4pluto_loaded && (nohup /tmp/sgchd > /tmp/sgchd.log 2>&1 < /dev/null &)" \
  || { echo "satcom4pluto-load: FPGA image did not load"; exit 1; }
sleep 2
pssh "pidof sgchd > /dev/null" || { echo "satcom4pluto-load: sgchd did not start:"; pssh "cat /tmp/sgchd.log"; exit 1; }
echo "satcom4pluto-load: done"
