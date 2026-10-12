#!/usr/bin/env python3
#
# SPDX-License-Identifier: MIT
#
r"""Control the Pluto satellite-downlink channelizer (sgchd on the Pluto) over TCP (port 30470).

Detector options are passed to sgchd unchanged (the same options as OpenWebRX's
"Detector options" field and DETECTOR= in satcom4pluto.bat; see README.md,
"Detector options", or `sgchd --help`). Detection is always on.

Examples
    # set the detector
    sgch_ctl.py --host 192.168.5.5 -f 240:270 --max-snr-db 40 --kernel sin3
    sgch_ctl.py --host 192.168.5.5 -f 240:270 --spur 250.0 --spur 262.144 --pass 248.9:249.27

    # status, open windows, tracked downlinks, current options, live view
    sgch_ctl.py --host 192.168.5.5 --status
    sgch_ctl.py --host 192.168.5.5 --channels
    sgch_ctl.py --host 192.168.5.5 --tracks
    sgch_ctl.py --host 192.168.5.5 --get
    sgch_ctl.py --host 192.168.5.5 --watch

    # diagnostics: no detection, or give the Pluto back to normal SDR programs
    sgch_ctl.py --host 192.168.5.5 -f 240:245 --all
    sgch_ctl.py --host 192.168.5.5 --default
"""

import argparse
import socket
import sys
import time

PORT = 30470


def talk(host, port, lines):
    s = socket.create_connection((host, port), timeout=5)
    s.sendall(("".join(l + "\n" for l in lines) + "QUIT\n").encode())
    data = b""
    while True:
        b = s.recv(65536)
        if not b:
            break
        data += b
    s.close()
    out = data.decode(errors="replace")
    return out.replace("OK bye\n", "")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                usage="%(prog)s [--host H] [--status|--channels|--tracks|--get|--watch] [DETECTOR OPTIONS ...]")
    p.add_argument("--host", default="192.168.2.1", help="Pluto address (default 192.168.2.1; yours: 192.168.5.5)")
    p.add_argument("--port", type=int, default=PORT, help="sgchd control port (default 30470)")
    p.add_argument("--status", action="store_true", help="show the daemon status")
    p.add_argument("--channels", action="store_true", help="list the open windows")
    p.add_argument("--tracks", action="store_true", help="list the tracked downlinks")
    p.add_argument("--get", action="store_true", help="show the current detector options")
    p.add_argument("--watch", action="store_true", help="show status, windows and tracks every second")
    a, det_opts = p.parse_known_args()

    if a.status or a.channels or a.get or a.watch or a.tracks:
        while True:
            cmds = []
            if a.get:
                cmds.append("GET")
            if a.status or a.watch:
                cmds.append("STATUS")
            if a.channels or a.watch:
                cmds.append("CHANNELS")
            if a.tracks or a.watch:
                cmds.append("TRACKS")
            out = talk(a.host, a.port, cmds)
            if a.watch:
                sys.stdout.write("\033[2J\033[H")
            for line in out.splitlines():
                if line.startswith("OK fs="):
                    for kv in line[3:].split():
                        print("  " + kv.replace("=", ": "))
                elif line.startswith("OK ") and line[3:].strip().isdigit():
                    print("%s windows (centre MHz, width kHz, peak SNR dB, state):" % line[3:])
                elif line.startswith("OK ") and " bin_hz=" in line:
                    print("%s tracks: id state centre_MHz bw_kHz snr_dB z conf age_s since_s parent indep "
                          "extract_lo_MHz extract_hi_MHz streaming" % line[3:].split()[0])
                else:
                    print(line)
            if not a.watch:
                return
            time.sleep(1)

    if not det_opts:
        p.print_help()
        return
    args = det_opts
    out = talk(a.host, a.port, ["SET " + " ".join(args)])
    print(out.strip())
    if out.startswith("ERR"):
        sys.exit(1)


if __name__ == "__main__":
    main()
