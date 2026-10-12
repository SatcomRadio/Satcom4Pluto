"""Floating-point model of the SGCH FPGA channelizer (rtl/sgc_core.v).

Analysis: frames of N samples, hop N/2, sqrt-Hann window w[n] = sin(pi n / N),
FFT (bin b = 0 is DC). Frame k covers samples k*N/2 .. k*N/2 + N - 1.

Detection spectrum: a short complex kernel across bins applied to X (equivalent
to multiplying the frame by sin^(2m) in time), |.|^2, summed over 2^avg frames.
  sin1 : [1]                   -> sqrt-Hann (sine) window
  sin3 : [-1 2 -1]/4           -> sin^3 window
  sin5 : [1 -4 6 -4 1]/16      -> sin^5 window (SGCH v1.0 fixed choice)
Synthesis (PC): IFFT + sqrt-Hann, overlap-add with hop N/2 (perfect
reconstruction when all bins are kept). Sub-band synthesis with decimation
D = N/M, see synth_slot().
"""
import numpy as np

KERNELS = {
    "sin1": np.array([1.0]),
    "sin3": np.array([-1.0, 2.0, -1.0]) / 4.0,
    "sin5": np.array([1.0, -4.0, 6.0, -4.0, 1.0]) / 16.0,
}


def window(N):
    return np.sin(np.pi * np.arange(N) / N)


def analysis(x, N):
    """WOLA analysis. Returns complex64 array [frames, N] (natural bin order)."""
    H = N // 2
    nfr = (len(x) - N) // H + 1
    w = window(N).astype(np.float32)
    out = np.empty((nfr, N), np.complex64)
    step = 512
    for k0 in range(0, nfr, step):
        k1 = min(nfr, k0 + step)
        idx = (np.arange(k0, k1)[:, None] * H + np.arange(N)[None, :])
        out[k0:k1] = np.fft.fft(x[idx] * w[None, :], axis=1)
    return out


def det_spectrum(F, kernel="sin5", avg_frames=256):
    """Averaged detection spectrum, one row per update: P[u, b] (linear)."""
    k = KERNELS[kernel]
    h = len(k) // 2
    if len(k) > 1:
        Y = np.zeros_like(F)
        for i, c in enumerate(k):
            Y += c * np.roll(F, h - i, axis=1)   # Y[b] = sum_i k[i] X[b + i - h]
    else:
        Y = F
    p = (Y.real.astype(np.float64) ** 2 + Y.imag.astype(np.float64) ** 2)
    nu = p.shape[0] // avg_frames
    return p[:nu * avg_frames].reshape(nu, avg_frames, -1).sum(1)


def centred(P):
    """natural bin order -> centred order (index N/2 = DC)."""
    return np.fft.fftshift(P, axes=-1)


def synth_full(F, gains):
    """Full-rate overlap-add synthesis of selected bins. gains: [frames, N] or [N]."""
    nfr, N = F.shape
    H = N // 2
    w = window(N)
    G = F * gains
    y = np.zeros((nfr + 1) * H, np.complex128)
    for k in range(nfr):
        y[k * H:k * H + N] += np.fft.ifft(G[k]) * w
    return y


def synth_slot(F, c, M, mask=None):
    """Decimated sub-band synthesis around bin c (natural order, may be negative).

    Uses bins c - M/2 .. c + M/2 - 1 (times mask[d], d = 0..M-1), output rate
    fs * M / N, baseband: bin c -> 0 Hz. Output sample m of frame k is at input
    time k*N/2 + m*D (D = N/M). Overlap-add hop M/2.
    """
    nfr, N = F.shape
    assert N % M == 0
    D = N // M
    ws = window(N)[::D]                     # synthesis window sampled at n = D m
    d = np.arange(-M // 2, M // 2)
    bins = (c + d) % N
    Z = F[:, bins].astype(np.complex128)
    if mask is not None:
        Z = Z * mask[None, :]
    # (1/N) sum_d X[c+d] e^{j 2 pi d m / M}: put d at index d mod M
    Zr = np.zeros((nfr, M), np.complex128)
    Zr[:, d % M] = Z
    seg = np.fft.ifft(Zr, axis=1) * (M / N)
    sign = (-1.0) ** ((c * np.arange(nfr)) % 2)   # e^{-j pi c k}
    seg = seg * sign[:, None] * ws[None, :]
    Hm = M // 2
    y = np.zeros((nfr + 1) * Hm, np.complex128)
    for k in range(nfr):
        y[k * Hm:k * Hm + M] += seg[k]
    return y
