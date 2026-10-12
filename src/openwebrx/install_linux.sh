#!/bin/sh
# satcom4pluto, Linux side. One command:   sudo sh install_linux.sh  [OpenWebRX container name | --native]
#  1. service satcom4pluto: loads the satcom4pluto FPGA image + detector into the Pluto's RAM after
#     every Pluto power-up (nothing is ever written to the Pluto's flash)
#  2. adds the "satcom4pluto" device type to OpenWebRX (install_openwebrx.sh)
set -e
cd "$(dirname "$0")"
D=/usr/local/share/satcom4pluto
mkdir -p $D
install -m 755 satcom4pluto-load.sh satcom4pluto.sh askpass.sh install_openwebrx.sh $D/
install -m 644 sgch_pl.bin pluto_tryload.sh libsatcom4plutoSupport.so satcom4pluto.py feature_patch.py $D/
install -m 755 sgchd $D/sgchd
[ -f /etc/satcom4pluto.conf ] || install -m 600 satcom4pluto.conf /etc/satcom4pluto.conf
install -m 644 satcom4pluto.service /etc/systemd/system/satcom4pluto.service
# ssh must log in with a password without a terminal: sshpass, or OpenSSH >= 8.4
command -v sshpass > /dev/null 2>&1 || {
  v=$(ssh -V 2>&1 | sed -n 's/^OpenSSH_\([0-9]*\)\.\([0-9]*\).*/\1\2/p')
  [ "${v:-0}" -ge 84 ] || echo "WARNING: install sshpass (apt install sshpass): this ssh is older than OpenSSH 8.4"
}
systemctl daemon-reload
systemctl enable --now satcom4pluto
echo "--- loading the Pluto ..."
. /etc/satcom4pluto.conf
$D/satcom4pluto-load.sh "$PLUTO" || echo "WARNING: Pluto not loaded yet (plugged in? ping $PLUTO) -- the service keeps trying"
echo "--- OpenWebRX ..."
sh $D/install_openwebrx.sh "${1:-}"
echo "--- done. Log: journalctl -u satcom4pluto -f"
