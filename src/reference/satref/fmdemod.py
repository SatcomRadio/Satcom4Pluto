"""FM demodulation and audio quality against the known modulating audio."""
import numpy as np
from fractions import Fraction
from scipy import signal as sps

from .signals import AUDIO_FS


def ddc(x, fs, f0, bw, fs_out):
    """DDC: mix f0 -> 0, decimate in stages (polyphase anti-alias), FIR low-pass bw/2,
    resample to fs_out."""
    n = np.arange(len(x))
    y = x * np.exp(-2j * np.pi * f0 * n / fs)
    while fs / 4 >= 4 * max(bw, fs_out):
        y = sps.resample_poly(y, 1, 4)
        fs = fs / 4
    while fs / 2 >= 4 * max(bw, fs_out):
        y = sps.resample_poly(y, 1, 2)
        fs = fs / 2
    h = sps.firwin(511, bw / 2, fs=fs)
    y = sps.oaconvolve(y, h, mode="same")
    r = Fraction(int(round(fs_out)), int(round(fs))).limit_denominator(100000)
    return sps.resample_poly(y, r.numerator, r.denominator)


def fm_demod(y, fs, dev):
    d = np.angle(y[1:] * np.conj(y[:-1])) * fs / (2 * np.pi * dev)
    return np.concatenate([[0.0], d])


def audio_from_iq(y, fs, dev):
    a = fm_demod(y, fs, dev)
    r = Fraction(AUDIO_FS, int(round(fs))).limit_denominator(10000)
    a = sps.resample_poly(a, r.numerator, r.denominator)
    sos = sps.butter(6, [300, 3000], btype="band", fs=AUDIO_FS, output="sos")
    return sps.sosfiltfilt(sos, a)
    r = Fraction(AUDIO_FS, int(round(fs))).limit_denominator(10000)
    return sps.resample_poly(a, r.numerator, r.denominator)


def audio_snr(rec, ref, t_skip=0.3, t_end=None):
    """SNR (dB) of rec vs ref after best delay/gain fit, band 300..3000 Hz, plus
    the fraction of 20 ms blocks with SNR below 0 dB (dropouts / clicks)."""
    sos = sps.butter(6, [300, 3000], btype="band", fs=AUDIO_FS, output="sos")
    ref = sps.sosfiltfilt(sos, ref)
    n = min(len(rec), len(ref))
    i0 = int(t_skip * AUDIO_FS)
    i1 = n if t_end is None else min(n, int(t_end * AUDIO_FS))
    seg = rec[i0:i1]
    best = None
    for d in range(-400, 401):
        r = ref[i0 - d:i1 - d] if i0 - d >= 0 and i1 - d <= len(ref) else None
        if r is None or len(r) != len(seg):
            continue
        c = np.dot(seg, r)
        if best is None or c > best[0]:
            best = (c, d)
    d = best[1]
    r0 = ref[i0 - d:i1 - d]
    # fractional delay refinement (FFT phase shift of the reference)
    Rf = np.fft.rfft(r0)
    fr = np.fft.rfftfreq(len(r0))
    bestf = None
    for delta in np.arange(-0.5, 0.501, 0.05):
        r = np.fft.irfft(Rf * np.exp(-2j * np.pi * fr * delta), len(r0))
        g = np.dot(seg, r) / np.dot(r, r)
        e = seg - g * r
        v = np.dot(e, e)
        if bestf is None or v < bestf[0]:
            bestf = (v, r, g, e)
    _, r, g, e = bestf
    snr = 10 * np.log10(np.dot(g * r, g * r) / max(np.dot(e, e), 1e-12))
    blk = int(0.02 * AUDIO_FS)
    bad = 0
    nb = 0
    for k in range(0, len(seg) - blk, blk):
        pr = np.dot(r[k:k + blk], r[k:k + blk]) * g * g
        if pr < 1e-6 * blk:
            continue
        nb += 1
        if 10 * np.log10(pr / max(np.dot(e[k:k + blk], e[k:k + blk]), 1e-12)) < 0:
            bad += 1
    return snr, bad / max(nb, 1), d, g
