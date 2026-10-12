"""Run detectors on scenarios and score them against the truth."""
import math

import numpy as np

from . import fpga_model as fm
from . import scenarios as scn
from .det_v15 import DetV15
from .det_v2 import DetV2, Cfg, STREAMING


def spectra(x, kernel, avg):
    F = fm.analysis(x.astype(np.complex64), scn.N)
    P = fm.centred(fm.det_spectrum(F, kernel, avg))
    return F, P


def truth_bins(tr):
    lo = scn.N / 2 + (tr["f"] - tr["b99"] / 2) / scn.BINW
    hi = scn.N / 2 + (tr["f"] + tr["b99"] / 2) / scn.BINW
    return lo, hi


def on_at(tr, t0, t1):
    return any(a <= t0 and t1 <= b for a, b in tr["on"])


def overlaps_any(tr, t0, t1):
    return any(a < t1 and t0 < b for a, b in tr["on"])


def score(truth, reports, masks, upd_s, ignore=()):
    """reports[u] = list of dict(fc, s, e, id) (centred bins); masks[u] = streamed bins."""
    U = len(reports)
    res = dict(per=[], fa=0, fa_updates=0, merges=0, splits=0, upd=U)
    for k, tr in enumerate(truth):
        lo, hi = truth_bins(tr)
        blo, bhi = int(math.floor(lo)), int(math.ceil(hi))
        onset = min(a for a, b in tr["on"])
        cov = []
        first = None
        ferr, berr = [], []
        nsplit = 0
        for u in range(U):
            t0, t1 = u * upd_s, (u + 1) * upd_s
            if not on_at(tr, t0, t1):
                continue
            m = masks[u][max(0, blo):bhi]
            c = m.mean() >= 0.9 if len(m) else False
            ov = [r for r in reports[u] if r["s"] <= hi and r["e"] >= lo]
            ok = c and len(ov) > 0
            cov.append(ok)
            if ok and first is None:
                first = t1
            if ov:
                best = min(ov, key=lambda r: abs(r["fc"] - (lo + hi) / 2))
                ferr.append(abs(best["fc"] - (scn.N / 2 + tr["f"] / scn.BINW)) * scn.BINW)
                berr.append(best["bw_hz"] - tr["b99"])
                # split: two primary reports whose nearest truth (by centre) is this one
                prim = [r for r in ov if not r.get("child")]
                if len(prim) >= 2:
                    cen_t = [(scn.N / 2 + o["f"] / scn.BINW) for o in truth if on_at(o, t0, t1)]
                    mine = scn.N / 2 + tr["f"] / scn.BINW
                    own = [r for r in prim if min(cen_t, key=lambda q: abs(q - r["fc"])) == mine]
                    if len(own) >= 2:
                        nsplit += 1
        # steady-state Pd: ignore the first 0.25 s after each switch-on
        steady = []
        uu = [u for u in range(U) if on_at(tr, u * upd_s, (u + 1) * upd_s)]
        for u, ok in zip(uu, cov):
            t0 = u * upd_s
            last_on = max(a for a, b in tr["on"] if a <= t0)
            if t0 - last_on >= 0.25:
                steady.append(ok)
        res["per"].append(dict(cls=tr["cls"], f=tr["f"], snr=tr["snr_db"], above=tr["above_floor_db"],
                               b99=tr["b99"], pd=float(np.mean(steady)) if steady else float("nan"),
                               pd_all=float(np.mean(cov)) if cov else float("nan"),
                               latency=(first - onset) if first is not None else float("nan"),
                               ferr=float(np.median(ferr)) if ferr else float("nan"),
                               berr=float(np.median(berr)) if berr else float("nan"),
                               split_frac=nsplit / max(1, len(cov))))
    # false alarms and merges
    for u in range(U):
        t0, t1 = u * upd_s, (u + 1) * upd_s
        fa_u = 0
        for r in reports[u]:
            hitt = []
            for tr in truth:
                lo, hi = truth_bins(tr)
                if r["s"] <= hi + 1 and r["e"] >= lo - 1 and overlaps_any(tr, t0 - 2.5, t1):
                    hitt.append(tr)
            if any(abs(r["fc"] - (scn.N / 2 + f / scn.BINW)) < 2 for f in ignore):
                continue
            if not hitt:
                fa_u += 1
            elif not r.get("child"):
                # merge: report contains the centres of two truths whose 99 % bands do not overlap
                cs = [tr for tr in hitt if r["s"] <= scn.N / 2 + tr["f"] / scn.BINW <= r["e"]
                      and on_at(tr, t0, t1)]
                for a in range(len(cs)):
                    for b in range(a + 1, len(cs)):
                        if abs(cs[a]["f"] - cs[b]["f"]) >= (cs[a]["b99"] + cs[b]["b99"]) / 2:
                            res["merges"] += 1
        res["fa"] += fa_u
        res["fa_updates"] += fa_u > 0
    return res


def run_v15(P, upd_s, **kw):
    d = DetV15(scn.N, scn.BINW, upd_s, **kw)
    reps, masks = [], []
    for u in range(P.shape[0]):
        dets, op = d.update(P[u])
        reps.append([dict(fc=r["fc"], s=r["s"], e=r["e"], bw_hz=(r["e"] - r["s"] + 1) * scn.BINW) for r in dets])
        masks.append(op)
    return reps, masks


def run_v2(P, upd_s, cfg=None, keep=False, kernel="sin5"):
    d = DetV2(scn.N, scn.BINW, upd_s, cfg, kernel=kernel)
    reps, masks, hist = [], [], []
    for u in range(P.shape[0]):
        tracks, runs, slots = d.update(P[u], t=(u + 1) * upd_s)
        r = []
        for t in tracks:
            if t.state in STREAMING:
                r.append(dict(fc=t.fc, s=t.lo_bin, e=t.hi_bin, bw_hz=t.bw * scn.BINW, id=t.id,
                              child=t.parent >= 0, state=t.state))
        m = np.zeros(scn.N, bool)
        for s, e in runs:
            m[s:e + 1] = True
        reps.append(r)
        masks.append(m)
        if keep:
            hist.append(dict(runs=[tuple(x) for x in runs], slots=slots,
                             tracks=[(t.id, t.state, t.fc, t.bw, t.parent, t.independent) for t in tracks]))
    return reps, masks, hist, d


def out_rate(masks, upd_s):
    """mean selected bins -> output words/s (bins * frames/s, ignoring headers)."""
    frames_s = scn.FS / (scn.N / 2)
    return float(np.mean([m.sum() for m in masks]) * frames_s)
