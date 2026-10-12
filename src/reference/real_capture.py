"""Detectors on a real Pluto capture (36 MS/s, LO 255 MHz, N = 8192, 4.5 kHz bins),
plus synthetic weak FM emitters injected into empty parts of the real spectrum.

usage: python3 real_capture.py CAPTURE.cs16 [inject]
"""
import sys, math, numpy as np
from satref import fpga_model as fm, signals as sg
from satref.det_v15 import DetV15
from satref.det_v2 import DetV2, Cfg, STREAMING

FS, LO, N = 36e6, 255e6, 8192
BINW = FS / N
AVG = 256
UPD = AVG * (N / 2) / FS
fn = sys.argv[1]
inject = len(sys.argv) > 2 and sys.argv[2] == "inject"

raw = np.memmap(fn, dtype=np.int16, mode="r")
ns = len(raw) // 2


def spectra(kernel, x_extra=None):
    """chunked FPGA-model spectra of the capture (+ optional additive signal)"""
    out = []
    chunk = AVG * (N // 2)
    k = 0
    tail = None
    while (k + 1) * chunk + N <= ns:
        a = raw[2 * k * chunk:2 * ((k + 1) * chunk + N // 2)].astype(np.float32)
        x = (a[0::2] + 1j * a[1::2]).astype(np.complex64)
        if x_extra is not None:
            x = x + x_extra(k * chunk, len(x)).astype(np.complex64)
        F = fm.analysis(x, N)[:AVG]
        out.append(fm.centred(fm.det_spectrum(F, kernel, AVG))[0])
        k += 1
    return np.array(out)


def emitters_fn(em):
    """em: list of (offset_Hz, class dev, fs_bb, amplitude, audio). Generate on demand."""
    cache = {}

    def f(i0, n):
        y = np.zeros(n, np.complex128)
        t = (np.arange(n) + i0) / FS
        for j, (off, dev, fsbb, amp) in enumerate(em):
            if j not in cache:
                aud = sg.audio(ns / FS + 0.2, "voice", seed=100 + j)
                cache[j] = sg.fm_baseband(aud, dev, fs_bb=fsbb)
            bb = cache[j]
            # nearest-sample hold of the baseband (fine for detection statistics)
            idx = np.minimum((t * fsbb).astype(np.int64), len(bb) - 1)
            y += amp * bb[idx] * np.exp(2j * np.pi * off * t)
        return y
    return f


def run(P, name):
    d15 = DetV15(N, BINW, UPD)
    d2 = DetV2(N, BINW, UPD, Cfg(), kernel=name)
    last15, last2 = None, None
    for u in range(P[name].shape[0] if isinstance(P, dict) else P.shape[0]):
        last15 = d15.update(P["sin5"][u])[0]
        last2 = d2.update(P[name][u], t=(u + 1) * UPD)
    return last15, d2


if __name__ == "__main__":
    P = {"sin5": spectra("sin5"), "sin3": spectra("sin3")}
    print("updates", P["sin3"].shape[0], "update %.1f ms" % (UPD * 1e3))
    d15s, d2 = run(P, "sin3")
    tr = [t for t in d2.tracks if t.state in STREAMING]
    print("v1.5 (3 dB, sin5): %d windows in the last update" % len(d15s))
    for r in d15s:
        print("   %.4f MHz  %4.1f dB" % ((LO + (r["fc"] - N / 2) * BINW) / 1e6, r["snr_db"]))
    print("v2 (sin3): %d streaming tracks" % len(tr))
    for t in sorted(tr, key=lambda t: t.fc):
        print("   id %3d %-9s %.4f MHz  bw %5.1f kHz  snr %5.1f dB  parent %d" % (
            t.id, t.state, (LO + (t.fc - N / 2) * BINW) / 1e6, t.bw * BINW / 1e3, t.snr_db, t.parent))
    np.save("real_P_sin3.npy", P["sin3"])
    if inject:
        # weak emitters in empty regions: in-band SNR from the measured local floor
        B = 10 ** (d2.Blog / 10)
        # noise power per bin of the capture (the det spectrum scale is irrelevant: use a
        # time-domain estimate from the long-run median of a quiet region's PSD)
        a = raw[:2 * 4 * 1048576].astype(np.float32)
        x = a[0::2] + 1j * a[1::2]
        f, p = __import__("scipy.signal").signal.welch(x, FS, nperseg=8192, return_onesided=False)
        f = np.fft.fftshift(f); p = np.fft.fftshift(p)
        occ = np.zeros(N, bool)
        for t in d2.tracks:
            occ[max(0, t.lo_bin - 4):t.hi_bin + 5] = True
        # empty stretches wide enough for a 40 kHz emitter (+ guard); the band is crowded,
        # so take the free gaps themselves rather than a fine grid
        cands, j = [], 400
        while j < N - 400:
            if not occ[j - 7:j + 8].any() and abs(j - N / 2) > 50:
                cands.append(j); j += 16
            else:
                j += 1
        print("free positions:", len(cands))
        rng = np.random.default_rng(5)
        rng.shuffle(cands)
        em = []
        truth = []
        specs = [("N5", 1500.0, 96000, 5000.0), ("N7", 3000.0, 96000, 7000.0), ("W40", 22000.0, 192000, 40000.0)]
        for i, snr in enumerate([-6, -4, -2, 0, 2][:max(1, len(cands) // 3)]):
            for c, dev, fsbb, b99 in specs:
                j = cands.pop()
                off = (j - N / 2) * BINW + 1100
                n0 = np.interp(off, f, p)                # noise PSD per Hz at that offset
                amp = math.sqrt(10 ** (snr / 10) * b99 * n0)
                em.append((off, dev, fsbb, amp))
                truth.append((c, snr, off))
        Pi = {"sin5": spectra("sin5", emitters_fn(em)), "sin3": spectra("sin3", emitters_fn(em))}
        d15s, d2 = run(Pi, "sin3")
        trk = [t for t in d2.tracks if t.state in STREAMING]
        print("injected: cls snr  above_floor | v1.5 hit | v2 hit (fc err Hz, bw kHz)")
        for c, snr, off in truth:
            jb = N / 2 + off / BINW
            h15 = any(abs(r["fc"] - jb) < 3 for r in d15s)
            m = [t for t in trk if t.lo_bin - 1 <= jb <= t.hi_bin + 1]
            s2 = ("yes (%+.0f, %.1f)" % ((m[0].fc - jb) * BINW, m[0].bw * BINW / 1e3)) if m else "no"
            print("   %-4s %+3d %5.1f dB | %-3s | %s" % (c, snr, 10 * math.log10(1 + 10 ** (snr / 10)), "yes" if h15 else "no", s2))
