"""Test scenarios on a sub-band simulation.

The FPGA splits 36.864 MS/s into N = 8192 bins of 4.5 kHz, hop 4096 (111 us).
A 4.608 MS/s, N = 1024 simulation has the same bin width, the same frame
duration and the same window, so every per-bin statistic is identical; it only
covers 4.6 MHz instead of 36.9 MHz. All frequencies are offsets from the
simulation centre.
"""
import numpy as np

from . import signals as sg

FS = 4.608e6
N = 1024
BINW = FS / N

CLASSES = {          # FM deviation, audio kind -> occupied (99 %) bandwidth
    "N5": dict(dev=1500.0, fs_bb=96000),     # ~5 kHz
    "N7": dict(dev=3000.0, fs_bb=96000),     # ~7 kHz
    "W25": dict(dev=12000.0, fs_bb=192000),  # ~22 kHz
    "W40": dict(dev=22000.0, fs_bb=192000),  # ~40 kHz
}


def emitter(cls, f, snr_db, on=None, fade=None, seed=0, kind="voice", spacing=None):
    return dict(cls=cls, f=f, snr_db=snr_db, on=on, fade=fade, seed=seed, kind=kind)


def build(sc, dur, seed=0):
    """Render a scenario: returns x (complex), truth list (with B99, audio)."""
    n = int(dur * FS)
    floor = sc.get("floor")
    x = sg.shaped_noise(n, FS, floor, seed=seed + 1000)
    truth = []
    for i, e in enumerate(sc.get("emitters", [])):
        p = CLASSES[e["cls"]]
        aud = sg.audio(dur + 0.1, e.get("kind", "voice"), seed=seed * 100 + i + e.get("seed", 0))
        bb = sg.fm_baseband(aud, p["dev"], fs_bb=p["fs_bb"])
        b99, _ = sg.occupied_bw(bb, p["fs_bb"])
        n0 = 10 ** (floor(np.array([e["f"]]))[0] / 10) if floor else 1.0
        amp = np.sqrt(10 ** (e["snr_db"] / 10) * b99 * n0)
        env = None
        if e.get("on"):
            env = sg.gate(n, FS, e["on"])
        if e.get("fade"):
            fe = sg.fading(n, FS, depth_db=e["fade"], rate_hz=1.0, seed=seed + i)
            env = fe if env is None else env * fe
        x += sg.place(bb, p["fs_bb"], FS, e["f"], n, amp=amp, env=env)
        truth.append(dict(i=i, cls=e["cls"], f=e["f"], b99=b99, snr_db=e["snr_db"], on=e.get("on") or [(0, dur)],
                          audio=aud, dev=p["dev"], amp=amp, n0=n0,
                          above_floor_db=10 * np.log10(1 + 10 ** (e["snr_db"] / 10))))
    for f, a_db in sc.get("spurs", []):
        t = np.arange(n) / FS
        x += np.sqrt(10 ** (a_db / 10) * BINW) * np.exp(2j * np.pi * f * t)
    if sc.get("dc"):
        x += np.sqrt(10 ** (sc["dc"] / 10) * BINW)
    return x, truth


def _ripple(slope_db_per_mhz=0.0, ripple_db=0.0, period_mhz=1.3):
    return lambda f: slope_db_per_mhz * f / 1e6 + ripple_db / 2 * np.sin(2 * np.pi * f / (period_mhz * 1e6))


def suite(weak=(0.0, -2.0, -4.0)):
    """Named scenarios. snr_db = in-band SNR (signal power / noise power in its 99 % band).
    A signal with in-band SNR 0 dB is 3 dB above the floor, -2.3 dB -> 2 dB, -5.9 dB -> 1 dB."""
    S = {}
    for snr in weak:
        S[f"weak_N5_{snr:+.0f}dB"] = dict(emitters=[emitter("N5", 101e3, snr), emitter("N5", -607e3, snr, seed=3),
                                                     emitter("N5", 1203e3, snr, seed=4)])
        S[f"weak_N7_{snr:+.0f}dB"] = dict(emitters=[emitter("N7", 151e3, snr), emitter("N7", -807e3, snr, seed=3),
                                                     emitter("N7", 1403e3, snr, seed=4)])
        S[f"weak_W40_{snr:+.0f}dB"] = dict(emitters=[emitter("W40", 201e3, snr), emitter("W40", -903e3, snr, seed=3),
                                                      emitter("W40", 1301e3, snr, seed=4)])
    S["adjacent_narrow"] = dict(emitters=[emitter("N5", 300e3, 8), emitter("N5", 310e3, 8, seed=2),
                                          emitter("N7", -500e3, 6), emitter("N7", -488e3, 6, seed=5)])
    S["adjacent_wide"] = dict(emitters=[emitter("W40", 300e3, 8), emitter("W40", 350e3, 8, seed=2),
                                        emitter("W25", -600e3, 8), emitter("W25", -575e3, 8, seed=5)])
    S["partial_overlap"] = dict(emitters=[emitter("W40", 300e3, 8), emitter("W40", 330e3, 4, seed=2,
                                                                             on=[(0.0, 1.0), (1.6, 3.0)]),
                                          emitter("W25", -600e3, 8), emitter("W25", -585e3, 8, seed=5)])
    S["narrow_in_wide"] = dict(emitters=[emitter("W40", 300e3, 6), emitter("N5", 309e3, 10, seed=2, kind="tones"),
                                         emitter("W40", -600e3, 6, seed=3), emitter("N7", -592e3, 4, seed=4,
                                                                                    on=[(0.3, 3.0)])])
    S["multi"] = dict(emitters=[emitter(c, f, s, seed=k) for k, (c, f, s) in enumerate([
        ("N5", -1900e3, 3), ("N7", -1500e3, 0), ("W40", -1200e3, 6), ("W25", -800e3, 2), ("N5", -400e3, -2),
        ("W40", 100e3, 15), ("N7", 520e3, 10), ("N5", 900e3, 20), ("W25", 1300e3, -1), ("N7", 1800e3, 4)])])
    S["bursty_fading"] = dict(emitters=[emitter("N5", 200e3, 3, on=[(0.2, 0.7), (1.2, 1.5), (2.0, 2.9)]),
                                        emitter("W40", -500e3, 4, fade=12),
                                        emitter("N7", 900e3, 2, on=[(0.5, 0.65), (1.0, 1.08), (1.8, 2.6)], seed=4),
                                        emitter("W25", -1300e3, 3, fade=8, on=[(0.4, 2.2)], seed=5)])
    S["variable_floor"] = dict(floor=_ripple(2.0, 4.0, 1.1),
                               emitters=[emitter("N5", -1500e3, 0), emitter("N7", -400e3, 0, seed=2),
                                         emitter("W40", 700e3, -2, seed=3), emitter("N5", 1700e3, 0, seed=4)])
    S["noise_only"] = dict(emitters=[])
    S["noise_only_ripple"] = dict(floor=_ripple(1.0, 3.0, 0.9), emitters=[])
    S["strong_adjacent"] = dict(emitters=[emitter("W40", 300e3, 45), emitter("N5", 345e3, 3, seed=2),
                                          emitter("N5", 380e3, 0, seed=3), emitter("W25", -500e3, 40, seed=4),
                                          emitter("N7", -540e3, 2, seed=5)])
    S["dc_spurs"] = dict(dc=30, spurs=[(450e3, 25), (-1100e3, 20), (1650e3, 12)],
                         emitters=[emitter("N5", 30e3, 3), emitter("N7", 470e3, 3, seed=2),
                                   emitter("W40", -1000e3, 2, seed=3)])
    return S
