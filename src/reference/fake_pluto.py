"""Fake Pluto for testing sgch_rx on the PC side without hardware:
iiod (30431, attributes only), sgchd control (30470: TRACKS/MODE/SUBSCRIBE/SET/STATUS/QUIT)
and the raw DMA word stream (30471) played in real time from gen_frames.py output.
usage: python3 fake_pluto.py words.bin sched.txt FS LO
"""
import socket, threading, sys, time, numpy as np
words = np.fromfile(sys.argv[1], dtype="<u4")
FS, LO = float(sys.argv[3]), float(sys.argv[4])
sched = []
cur = None
for line in open(sys.argv[2]):
    if line.startswith("AT "):
        cur = [int(line[3:]), ""]
    elif line.strip() == "END":
        sched.append(cur); cur = None
    elif cur is not None:
        cur[1] += line
state = {"pos": 0, "tracks": "OK 0 fs=%.0f lo=%.0f bin_hz=%.3f t=0\n" % (FS, LO, FS / 1024)}
attrs = {"sampling_frequency": str(int(FS)), "frequency": str(int(LO)), "hardwaregain": "30.000000 dB",
         "gain_control_mode": "manual", "rf_bandwidth": "40000000"}

def serve(port, fn):
    ls = socket.socket(); ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ls.bind(("127.0.0.1", port)); ls.listen(4)
    while True:
        c, _ = ls.accept()
        threading.Thread(target=fn, args=(c,), daemon=True).start()

def iiod(c):
    f = c.makefile("rb")
    while True:
        l = f.readline()
        if not l: return
        p = l.decode().split()
        if p[0] == "READ":
            v = attrs.get(p[-1], "0").encode() + b"\0"
            c.sendall(b"%d\n" % len(v) + v + b"\n")
        elif p[0] == "WRITE":
            n = int(p[-1]); d = f.read(n)
            attrs[p[-2]] = d.rstrip(b"\0").decode(); c.sendall(b"%d\n" % n)
        else:
            c.sendall(b"-22\n")

def ctl(c):
    f = c.makefile("rb")
    while True:
        l = f.readline()
        if not l: return
        cmd = l.decode().strip().split(" ")[0].upper()
        if cmd == "TRACKS": c.sendall(state["tracks"].encode())
        elif cmd == "QUIT": c.sendall(b"OK bye\n"); c.close(); return
        elif cmd == "STATUS": c.sendall(b"OK fake\n")
        else: c.sendall(b"OK\n")

def stream(c):
    frames_s = FS / 512.0
    t0 = time.time(); pos = 0; k = 0
    while pos < len(words):
        # real-time pacing: about one block of 1/50 s
        nxt = min(len(words), pos + int(len(words) / (len(words) / 1) * 0) + 4096 * 8)
        blk = words[pos:nxt]
        c.sendall(blk.tobytes())
        pos = nxt
        state["pos"] = pos
        while k < len(sched) and sched[k][0] <= pos:
            state["tracks"] = sched[k][1]; k += 1
        # pace by frames: count magic words sent
        nfr = int(np.count_nonzero(words[:pos] == 0x53474348)) if pos % (4096 * 64) == 0 else None
        time.sleep(max(0.0, (pos / len(words)) * 3.0 - (time.time() - t0)))
    time.sleep(1.0)
    c.close()

for p, fn in ((30431, iiod), (30470, ctl), (30471, stream)):
    threading.Thread(target=serve, args=(p, fn), daemon=True).start()
print("fake pluto ready", flush=True)
time.sleep(1e9)
