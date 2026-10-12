"""Synthetic test signals: audio, FM transmitters, shaped noise floor, spurs."""
import numpy as np
from scipy import signal as sps

AUDIO_FS = 16000


def audio(dur, kind="voice", seed=0, fs=AUDIO_FS):
    """Known audio waveform, peak ~1, band 300..3000 Hz."""
    rng = np.random.default_rng(seed)
    n = int(dur * fs)
    t = np.arange(n) / fs
    if kind == "tones":
        f = rng.choice([430, 710, 1030, 1470, 1910, 2390], 3, replace=False)
        a = sum(np.sin(2 * np.pi * fi * t + rng.uniform(0, 6.28)) for fi in f)
    else:  # speech-like: band-limited noise with syllabic (3-6 Hz) envelope + pitch harmonics
        b, aa = sps.butter(4, [300, 3000], btype="band", fs=fs)
        nz = sps.lfilter(b, aa, rng.standard_normal(n))
        f0 = 110 + 40 * rng.random()
        harm = sum(np.sin(2 * np.pi * k * f0 * t * (1 + 0.03 * np.sin(2 * np.pi * 0.7 * t))) / k for k in range(2, 20))
        harm = sps.lfilter(b, aa, harm)
        env = 0.5 + 0.5 * np.sin(2 * np.pi * (3 + 2 * rng.random()) * t + rng.uniform(0, 6))
        env = np.clip(env, 0, 1) ** 1.5
        a = env * (0.6 * nz / nz.std() + 0.6 * harm / harm.std())
    a = a / (np.max(np.abs(a)) + 1e-12)
    if kind != "tones":
        a = np.tanh(3.0 * a) / np.tanh(3.0)      # transmitter audio compression / limiter
    return a


def fm_baseband(aud, dev_hz, fs_a=AUDIO_FS, fs_bb=96000, preemph=False):
    """FM modulate audio (peak 1 -> dev_hz). Returns complex baseband at fs_bb."""
    up = sps.resample_poly(aud, fs_bb, fs_a)
    ph = 2 * np.pi * dev_hz * np.cumsum(up) / fs_bb
    return np.exp(1j * ph)


def place(bb, fs_bb, fs, f_off, n_out, t0=0.0, amp=1.0, env=None):
    """Upsample complex baseband to fs, shift to f_off, start at t0 seconds."""
    from fractions import Fraction
    r = Fraction(int(fs), int(fs_bb)).limit_denominator(10000)
    x = sps.resample_poly(bb, r.numerator, r.denominator)
    i0 = int(round(t0 * fs))
    out = np.zeros(n_out, np.complex128)
    m = min(len(x), n_out - i0)
    if m <= 0:
        return out
    t = (np.arange(m) + i0) / fs
    seg = amp * x[:m] * np.exp(2j * np.pi * f_off * t)
    if env is not None:
        seg *= env[i0:i0 + m]
    out[i0:i0 + m] = seg
    return out


def occupied_bw(bb, fs_bb, frac=0.99):
    f, p = sps.welch(bb, fs_bb, nperseg=4096, return_onesided=False)
    i = np.argsort(f)
    f, p = f[i], p[i]
    c = np.cumsum(p) / p.sum()
    lo = f[np.searchsorted(c, (1 - frac) / 2)]
    hi = f[np.searchsorted(c, 1 - (1 - frac) / 2)]
    return hi - lo, (hi + lo) / 2


def shaped_noise(n, fs, floor_db_fn, seed=0):
    """Complex Gaussian noise with PSD 10^(floor_db_fn(f)/10) per Hz (f in Hz, -fs/2..fs/2)."""
    rng = np.random.default_rng(seed)
    w = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) * np.sqrt(fs / 2)
    if floor_db_fn is None:
        return w
    W = np.fft.fft(w)
    f = np.fft.fftfreq(n, 1 / fs)
    W *= 10 ** (floor_db_fn(f) / 20)
    return np.fft.ifft(W)


def gate(n, fs, on_off, ramp_s=0.005):
    """Envelope 0/1 from a list of (t_on, t_off) with short ramps."""
    e = np.zeros(n)
    t = np.arange(n) / fs
    for a, b in on_off:
        e = np.maximum(e, np.clip(np.minimum((t - a) / ramp_s, (b - t) / ramp_s), 0, 1))
    return e


def fading(n, fs, depth_db=10, rate_hz=0.5, seed=0):
    """Slow log-normal-ish fading envelope (amplitude)."""
    rng = np.random.default_rng(seed)
    k = max(4, int(n / fs * rate_hz * 4))
    pts = rng.standard_normal(k)
    x = np.interp(np.linspace(0, k - 1, n), np.arange(k), pts)
    db = -depth_db / 2 + depth_db / 2 * np.tanh(x)
    return 10 ** (db / 20)
