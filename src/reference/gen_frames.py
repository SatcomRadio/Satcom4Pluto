"""End-to-end test of the PC channel slots (sw/pc/channels.cpp) on FPGA-format data.

scenario -> FPGA analysis (fixed-point output words like sgc_core: osh, 16-bit
saturation) -> det_v2 decisions (runs, one update late as on the Pluto) -> DMA word
stream + sgchd TRACKS replies -> chantest (C++ Receiver + ChannelMux) -> slot IQ
files -> FM demodulator -> audio SNR vs the known audio.

usage: python3 gen_frames.py [slot_bins]
"""
import sys, os, subprocess, math, numpy as np
from satref import scenarios as scn, fpga_model as fm, fmdemod as dm
from satref.det_v2 import DetV2, Cfg, STREAMING

AVG = 256
UPD = AVG * (scn.N / 2) / scn.FS
LO = 255e6
L = int(math.log2(scn.N))
M_SLOT = int(sys.argv[1]) if len(sys.argv) > 1 else 16
OSH = 0

sc = dict(emitters=[scn.emitter("N5", -1203e3, 20), scn.emitter("N7", -402e3, 20, seed=2),
                    scn.emitter("W40", 398e3, 20, seed=3), scn.emitter("N7", 1201e3, 6, seed=4),
                    scn.emitter("W25", 1500e3, 15, seed=5, on=[(0.6, 3.0)])])
x, truth = scn.build(sc, 3.0, seed=21)
# ADC units: noise ~ 10 LSB rms per component
x = x / np.sqrt(scn.FS) * 10.0
F = fm.analysis(x.astype(np.complex64), scn.N)
# FPGA units: X = FFT(x_adc * w) / 2^(L-10)
F = F / 2 ** (L - 10)
P = fm.centred(fm.det_spectrum(F, "sin3", AVG))
nfr = F.shape[0]
d = DetV2(scn.N, scn.BINW, UPD, Cfg(), kernel="sin3")
words = []
sched = []
sel = np.zeros(scn.N, bool)          # centred selection in force
sat = 0
for u in range(P.shape[0] + 1):
    f0, f1 = u * AVG, min(nfr, (u + 1) * AVG)
    # frames of this update with the selection decided at the end of the previous one
    nat = np.fft.ifftshift(sel)
    runs = []
    b = 0
    while b < scn.N:
        if nat[b]:
            s = b
            while b < scn.N and nat[b]:
                b += 1
            runs.append((s, b - s))
        else:
            b += 1
    for k in range(f0, f1):
        words += [0x53474348, k, (OSH << 28) | (L << 24) | len(runs)]
        for s, ln in runs:
            words.append((ln << 16) | s)
            v = F[k, s:s + ln] / 2 ** OSH
            re = np.clip(np.round(v.real), -32768, 32767).astype(np.int64)
            im = np.clip(np.round(v.imag), -32768, 32767).astype(np.int64)
            sat += int(np.sum(np.abs(v.real) > 32767) + np.sum(np.abs(v.imag) > 32767))
            words += list(((im & 0xFFFF) << 16) | (re & 0xFFFF))
    if u >= P.shape[0] or f1 >= nfr:
        break
    tracks, rr, slots = d.update(P[u], t=(u + 1) * UPD)
    sel[:] = False
    for s, e in rr:
        sel[s:e + 1] = True
    # TRACKS reply as sgchd prints it
    lines = ["OK %d fs=%.0f lo=%.0f bin_hz=%.3f t=%.3f" % (len(tracks), scn.FS, LO, scn.BINW, (u + 1) * UPD)]
    for t in tracks:
        det_hz = t.bw * scn.BINW
        ext = max(det_hz * d.c.bw_factor, d.c.narrow_extract_hz if det_hz < 12000 else 0) + 2 * d.c.guard_hz
        half = ext / 2 / scn.BINW
        s = max(2, int(math.floor(t.fc - half)))
        e = min(scn.N - 3, int(math.ceil(t.fc + half)))
        lines.append("%d %s %.6f %.3f %.1f %.1f %.2f %.2f %.2f %d %d %.6f %.6f %d" % (
            t.id, t.state, (LO + (t.fc - scn.N / 2) * scn.BINW) / 1e6, det_hz / 1e3, t.snr_db, t.z, t.conf,
            0, 0, t.parent, int(t.independent), (LO + (s - scn.N / 2 - 0.5) * scn.BINW) / 1e6,
            (LO + (e - scn.N / 2 + 0.5) * scn.BINW) / 1e6, int(t.state in STREAMING)))
    sched.append((len(words), "\n".join(lines)))
np.array(words, dtype="<u4").tofile("/tmp/words.bin")
with open("/tmp/sched.txt", "w") as f:
    for at, txt in sched:
        f.write("AT %d\n%s\nEND\n" % (at, txt))
print("frames %d words %d (%.0f words/s) saturated %d" % (nfr, len(words), len(words) / (nfr * scn.N / 2 / scn.FS), sat))
for fn in os.listdir("/tmp"):
    if fn.startswith("slot") and fn.endswith(".cf32"):
        os.remove("/tmp/" + fn)
r = subprocess.run(["/tmp/chantest", "/tmp/words.bin", "/tmp/sched.txt", "/tmp/slot", str(M_SLOT)],
                   capture_output=True, text=True)
print(r.stdout)
tab = [l.split() for l in r.stdout.splitlines() if l and l[0] != "#"]
fs_slot = scn.FS * M_SLOT / scn.N
print("emitter       SNR | ideal DDC | C++ slot (bad 20 ms) | slot  centre error")
for tr in truth:
    f_abs = LO + tr["f"]
    bwx = 2 * (tr["dev"] + 3000) * 1.1
    fs_ref = 48000 if tr["dev"] < 5000 else 96000
    a_ref = dm.audio_from_iq(dm.ddc(x, scn.FS, tr["f"], bwx, fs_ref), fs_ref, tr["dev"])
    s_ref = dm.audio_snr(a_ref, tr["audio"], 0.7)
    best = None
    for row in tab:
        if row[3] == "-":
            continue
        fcen = float(row[3]) * 1e6
        if best is None or abs(fcen - f_abs) < abs(best[1] - f_abs):
            best = (int(row[0]), fcen)
    if best is None or abs(best[1] - f_abs) > 20e3:
        print("%-4s %+8.0f %4.0f | %6.1f dB | not in a slot" % (tr["cls"], tr["f"], tr["snr_db"], s_ref[0]))
        continue
    y = np.fromfile("/tmp/slot%d.cf32" % best[0], dtype=np.complex64).astype(np.complex128)
    # frames before the first track list are not synthesised: realign to t = 0
    y = np.concatenate([np.zeros(nfr * M_SLOT // 2 - len(y) + M_SLOT // 2), y])
    # the slot output is centred on the advertised frequency: SDRangel's channel at offset
    # (true - advertised); demodulate with the same Carson filter as the reference
    off = f_abs - best[1]
    n = np.arange(len(y))
    yb = y * np.exp(-2j * np.pi * off * n / fs_slot)
    from scipy import signal as sps
    if bwx < fs_slot:
        yb = sps.oaconvolve(yb, sps.firwin(255, bwx / 2, fs=fs_slot), mode="same")
    a2 = dm.audio_from_iq(yb, fs_slot, tr["dev"])
    s2 = dm.audio_snr(a2, tr["audio"], 0.7)
    print("%-4s %+8.0f %4.0f | %6.1f dB | %6.1f dB (%4.2f)     | %d  %+.0f Hz" % (
        tr["cls"], tr["f"], tr["snr_db"], s_ref[0], s2[0], s2[1], best[0], best[1] - f_abs))
