"""PC chain over real sockets: fake_pluto.py -> sgch_rx --channels -> sdra_bench (as SDRangel)
-> FM demod -> audio SNR. Needs /tmp/words.bin, /tmp/sched.txt from gen_frames.py."""
import subprocess, time, sys, os, re, numpy as np
from scipy import signal as sps
sys.path.insert(0, ".")
src = open("gen_frames.py").read().split("F = fm.analysis")[0]
exec(src)                                     # truth, x (scenario), constants
FSL = 72000
fp = subprocess.Popen(["python3", "fake_pluto.py", "/tmp/words.bin", "/tmp/sched.txt", str(scn.FS), str(LO)],
                      stdout=subprocess.PIPE, text=True)
fp.stdout.readline()
rx = subprocess.Popen(["/tmp/sgch_rx", "-u", "ip:127.0.0.1", "--channels", "6", "--base-port", "1235"],
                      stderr=subprocess.PIPE, text=True)
time.sleep(0.4)
benches = [subprocess.Popen(["/tmp/sdra_bench", str(1235 + k), "4.0", "/tmp/e2e_slot%d.cs16" % k],
                            stdout=subprocess.PIPE, text=True) for k in range(6)]
outs = [b.communicate()[0] for b in benches]
time.sleep(0.5)
rx.terminate(); err = rx.communicate()[1]; fp.terminate()
print("\n".join(l for l in err.splitlines()[-14:]))
for k, o in enumerate(outs):
    print("slot %d:" % k, " | ".join(o.strip().splitlines()))
# frequency announced last per slot
freq = {}
for k, o in enumerate(outs):
    m = re.findall(r"setCenterFrequency ([0-9.]+) MHz", o) or re.findall(r"device \d+, ([0-9.]+) MHz", o)
    if m and abs(float(m[-1]) * 1e6 - LO) > 1: freq[k] = float(m[-1]) * 1e6
print("emitter     SNR | ideal DDC | via TCP slot (bad) | slot")
for tr in truth:
    f_abs = LO + tr["f"]
    k = min(freq, key=lambda q: abs(freq[q] - f_abs)) if freq else None
    bwx = 2 * (tr["dev"] + 3000) * 1.1
    fs_ref = 48000 if tr["dev"] < 5000 else 96000
    if k is None or abs(freq[k] - f_abs) > 20e3:
        print("%-4s %+8.0f %4.0f | not in a slot" % (tr["cls"], tr["f"], tr["snr_db"])); continue
    q = np.fromfile("/tmp/e2e_slot%d.cs16" % k, dtype="<i2").astype(float)
    y = q[0::2] + 1j * q[1::2]
    nz = np.nonzero(np.abs(y) > 0)[0]
    y = y[nz[0]:] if len(nz) else y
    off = f_abs - freq[k]
    n = np.arange(len(y))
    yb = y * np.exp(-2j * np.pi * off * n / FSL)
    if bwx < FSL:
        yb = sps.oaconvolve(yb, sps.firwin(255, bwx / 2, fs=FSL), mode="same")
    a2 = dm.audio_from_iq(yb, FSL, tr["dev"])
    # coarse alignment by cross-correlation with the known audio, then the usual metric
    ref = tr["audio"]
    seg = a2[3200:3200 + 16000] if len(a2) > 20000 else a2
    c = sps.correlate(ref, seg, mode="valid", method="fft")
    lag = int(np.argmax(c))
    pad = lag - 3200
    a2a = np.concatenate([np.zeros(max(0, pad)), a2[max(0, -pad):]])
    s2 = dm.audio_snr(a2a, ref, 1.0, 2.8)
    print("%-4s %+8.0f %4.0f | (see gen) | %6.1f dB (%4.2f) | %d" % (tr["cls"], tr["f"], tr["snr_db"], s2[0], s2[1], k))
