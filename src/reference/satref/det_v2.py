"""satcom4pluto v2 detector: multiresolution adaptive detection, segmentation,
narrow-in-wide features, temporal tracking and IQ interval allocation.

Reference implementation for sgchd (sw/arm/sgchd.c, det2_*). Every step uses
simple loops / sorts so that the C port is direct. All spectra are linear power
in centred bin order (index N/2 = LO).

Per update:
  1. baseline B[j]  : masked segment percentile (bias-corrected), neighbour
                      median, log-linear interpolation, slow EMA in dB
  2. R = P / B      : normalised spectrum (noise -> 1);  Rs = EMA of R (slow)
  3. sigma_W        : empirical (MAD) std of the W-bin mean of R and Rs, per W
  4. z_W[j]         : (mean_W(R) - 1) / sigma_W  for every window width W
  5. greedy CLEAN   : strongest window -> seed -> extent by edge search with
                      valley stops (split adjacent signals) -> claim bins ->
                      re-test remaining windows on the residual
  6. narrow-in-wide : bins standing out of the local envelope inside wide signals
  7. tracker        : association, CANDIDATE->CONFIRMED->ACTIVE->HOLD->EXPIRED
  8. allocation     : extraction interval per streaming track, merged runs
"""
import math
from dataclasses import dataclass, field

import numpy as np

CAND, CONF, ACTIVE, HOLD, EXPIRED = "CANDIDATE", "CONFIRMED", "ACTIVE", "HOLD", "EXPIRED"
STREAMING = (CONF, ACTIVE, HOLD)


@dataclass
class Cfg:
    widths_hz: tuple = (5000, 7000, 12500, 25000, 40000, 60000)
    seg: int = 32                 # baseline segment (bins), hop seg/2
    q: float = 0.30               # baseline percentile
    nbr: int = 3                  # neighbour median over +-nbr segments
    base_alpha: float = 0.25      # EMA of the baseline (dB domain) per update
    slow_alpha: float = 0.15      # EMA of R for the slow (long-integration) spectrum
    z_det: float = 5.5            # new detection (fast spectrum)
    z_det_slow: float = 6.0       # new detection (slow spectrum)
    z_keep: float = 3.0           # existing track continues (hysteresis)
    z_edge: float = 2.0           # extent edge (z of the smoothed excess)
    edge_frac: float = 0.001      # ... or this fraction of the peak excess (-30 dB)
    split_ratio: float = 0.5      # valley excess <= ratio * smaller peak excess -> split
    split_z: float = 3.0          # and the dip must be significant (z)
    narrow_max_bins: int = 2      # narrow-in-wide feature width
    narrow_env: int = 7           # envelope median length (bins)
    narrow_db: float = 1.5        # feature must exceed the envelope by this much
    z_narrow: float = 6.0
    confirm_m: int = 2            # m of n updates to confirm
    confirm_n: int = 3
    z_fast_confirm: float = 15.0  # confirm at once
    active_hits: int = 5
    hold_s: float = 2.0
    child_confirm: int = 6        # narrow child: persistent updates before confirmation
    child_corr_max: float = 0.7   # child independent if power correlation with parent below
    stream_candidates: bool = False
    dc_notch_hz: float = 5000.0
    spur_hz: tuple = ()           # known spurs (offset from LO, Hz), masked +-1 bin
    guard_hz: float = 4500.0      # extraction guard each side
    bw_factor: float = 1.25       # extraction = detected bw * factor + 2 guard
    narrow_extract_hz: float = 12500.0   # minimum extraction for narrow signals
    merge_gap_bins: int = 2
    split_mature: int = 7         # a valley splits only after this many updates of slow integration
    max_bw_hz: float = 150000.0   # a track never grows wider (dense bands: no runaway merging)


@dataclass
class Track:
    id: int
    fc: float                     # bin (centred, fractional)
    bw: float                     # bins (detected extent)
    snr_db: float
    z: float
    t_first: float
    t_last: float
    state: str = CAND
    hits: list = field(default_factory=list)   # 1/0 history (recent)
    nhits: int = 0
    parent: int = -1
    independent: bool = True
    merged: bool = False
    fhist: list = field(default_factory=list)
    bhist: list = field(default_factory=list)
    phist: list = field(default_factory=list)  # excess power history (for child validation)
    lo_bin: int = 0
    hi_bin: int = 0
    conf: float = 0.0
    ext: float = 1.0               # half extent of the detected emission (bins)
    sep_hits: int = 0
    absorb: int = 0


def _movmean(x, W):
    """mean over x[a:a+W] with a = j - (W-1)//2, edges -> nan."""
    N = len(x)
    c = np.concatenate([[0.0], np.cumsum(x)])
    a = np.arange(N) - (W - 1) // 2
    b = a + W
    ok = (a >= 0) & (b <= N)
    out = np.full(N, np.nan)
    out[ok] = (c[b[ok]] - c[a[ok]]) / W
    return out


def _mad_sigma(v):
    v = v[np.isfinite(v)]
    if len(v) < 16:
        return np.nan
    m = np.median(v)
    return 1.4826 * np.median(np.abs(v - m))


class DetV2:
    SMEAR = {"sin1": 2.6, "sin3": 3.5, "sin5": 4.4}

    def __init__(self, N, binw, update_s, cfg=None, kernel="sin5"):
        self.smear = self.SMEAR[kernel]
        c0 = cfg or Cfg()
        self.cap_bw = c0.max_bw_hz / binw
        self.cap_ext = self.cap_bw / 2 + self.smear / 2 + 1
        self.N, self.binw, self.dt = N, binw, update_s
        self.c = cfg or Cfg()
        self.Wb = sorted(set(max(1, int(round(w / binw))) for w in self.c.widths_hz))
        self.Blog = None
        self.Rs = None
        self.age = np.zeros(N, int)
        self.sig_f = {}
        self.sig_s = {}
        self.tracks = []
        self.next_id = 1
        self.t = 0.0
        self.fixed_mask = np.zeros(N, bool)
        dc = int(math.ceil(self.c.dc_notch_hz / binw))
        self.fixed_mask[N // 2 - dc:N // 2 + dc + 1] = True
        for f in self.c.spur_hz:
            j = int(round(N / 2 + f / binw))
            self.fixed_mask[max(0, j - 1):min(N, j + 2)] = True
        self.fixed_pos = [N // 2] if self.c.dc_notch_hz > 0 else []
        self.fixed_pos += [int(round(N / 2 + f / binw)) for f in self.c.spur_hz]
        self.fixed_mask[:4] = True
        self.fixed_mask[-4:] = True

    # ------------------------------------------------------------------ baseline
    def _baseline(self, P, mask, sig1):
        N, S = self.N, self.c.seg
        hop = S // 2
        starts = list(range(0, N - S + 1, hop))
        cen = np.array([s + S / 2 - 0.5 for s in starts])
        v = np.full(len(starts), np.nan)
        zq = -0.5244  # Gaussian quantile of q = 0.30
        corr = 1.0 + zq * min(sig1, 0.5)
        for i, s in enumerate(starts):
            seg = P[s:s + S][~mask[s:s + S]]
            if len(seg) >= S // 4:
                v[i] = np.quantile(seg, self.c.q) / corr
        lv = 10 * np.log10(v)
        good = np.isfinite(lv)
        if good.sum() < 3:
            lv[:] = 10 * np.log10(np.median(P) + 1e-30)
        else:
            lv[~good] = np.interp(cen[~good], cen[good], lv[good])
        # robust against segments fully inside a strong emission: clip each segment to
        # the median of the symmetric neighbour-pair averages (cancels a linear slope)
        k = self.c.nbr
        lv2 = lv.copy()
        for i in range(len(lv)):
            pr = [(lv[i - d] + lv[i + d]) / 2 for d in range(1, k + 1) if i - d >= 0 and i + d < len(lv)]
            if pr:
                lv2[i] = min(lv[i], float(np.median(pr)) + 1.0)
        out = np.interp(np.arange(N), cen, lv2)
        # linear extrapolation beyond the first / last segment centre
        j0 = np.arange(N) < cen[0]
        j1 = np.arange(N) > cen[-1]
        out[j0] = lv2[0] + (np.arange(N)[j0] - cen[0]) * (lv2[1] - lv2[0]) / (cen[1] - cen[0])
        out[j1] = lv2[-1] + (np.arange(N)[j1] - cen[-1]) * (lv2[-1] - lv2[-2]) / (cen[-1] - cen[-2])
        return out

    # ------------------------------------------------------------------ main
    def update(self, P, t=None):
        c, N = self.c, self.N
        self.t = self.t + self.dt if t is None else t
        # masks: fixed + current track extents
        mask = self.fixed_mask.copy()
        for tr in self.tracks:
            if tr.state != EXPIRED:
                mask[max(0, tr.lo_bin - 1):min(N, tr.hi_bin + 2)] = True
        # strong candidates of the previous update are masked too (one pass per update)
        for s_, e_ in getattr(self, "prev_strong", []):
            mask[max(0, s_ - 1):min(N, e_ + 2)] = True
        sig1 = self.sig_f.get(1, 1.0 / math.sqrt(64))
        Bl = self._baseline(P, mask, sig1)
        Bnew = Bl if self.Blog is None else self.Blog + c.base_alpha * (Bl - self.Blog)
        B = 10 ** (Bnew / 10)
        R = P / B
        Rs = R.copy() if self.Rs is None else self.Rs + c.slow_alpha * (R - self.Rs)
        sf, ss = self._sigmas(R, Rs, mask)
        # integration age of every bin's excess (splits need a mature slow spectrum)
        zs1 = (Rs - 1.0) / self._sigma_any(ss, 1) if 1 in self.Wb else (Rs - 1.0) / (ss[self.Wb[0]] * math.sqrt(self.Wb[0]))
        self.age_now = np.where(zs1 > c.z_edge, self.age + 1, 0)
        cands = self._clean(R, Rs, sf, ss, mask)
        self.prev_strong = [(cd["s"], cd["e"]) for cd in cands if cd["z"] > 2 * c.z_det]
        out = (Bnew, R, Rs, sf, ss, cands)
        self.Blog, R, self.Rs, self.sig_f, self.sig_s, cands = out
        self.age = self.age_now
        self.R = R
        cands += self._narrow(R, self.Rs, cands)
        self._track(R, self.Rs, cands)
        runs, slots = self._allocate()
        self.cands = cands
        return self.tracks, runs, slots

    def _sigmas(self, R, Rs, mask):
        sf, ss = {}, {}
        valid = ~mask
        for W in self.Wb:
            okw = _movmean(valid.astype(float), W) > 0.999
            okw &= (np.arange(self.N) % max(4, W)) == 0      # decimate: speed (ARM), less correlation
            mf = _movmean(R, W)
            ms = _movmean(Rs, W)
            s1 = _mad_sigma(mf[okw])
            s2 = _mad_sigma(ms[okw])
            # temporal smoothing of the estimate (stable thresholds)
            if W in self.sig_f and np.isfinite(s1):
                s1 = self.sig_f[W] + 0.2 * (s1 - self.sig_f[W])
            if W in self.sig_s and np.isfinite(s2):
                s2 = self.sig_s[W] + 0.2 * (s2 - self.sig_s[W])
            sf[W] = s1 if np.isfinite(s1) else 0.1
            ss[W] = s2 if np.isfinite(s2) else 0.1
        sf[1] = sf.get(1, sf[self.Wb[0]] * math.sqrt(self.Wb[0]))
        return sf, ss

    def _sigma_any(self, sd, W):
        """sigma for an arbitrary width W (log-log interpolation of the table)."""
        ws = np.array(sorted(sd))
        vs = np.array([sd[w] for w in ws])
        if W <= ws[0]:
            return vs[0] * math.sqrt(ws[0] / W)
        if W >= ws[-1]:
            return vs[-1] * math.sqrt(ws[-1] / W)
        return float(np.exp(np.interp(math.log(W), np.log(ws), np.log(vs))))

    # ------------------------------------------------------------------ CLEAN
    def _clean(self, R, Rs, sf, ss, mask):
        c, N = self.c, self.N
        Ef = R - 1.0
        Es = Rs - 1.0
        wins = []   # (z, W, a, slow?)
        for W in self.Wb:
            for E, sd, thr, slow in ((Ef, sf, c.z_det, 0), (Es, ss, c.z_det_slow, 1)):
                m = _movmean(E, W)
                z = m / sd[W]
                a = np.arange(N) - (W - 1) // 2
                ok = np.isfinite(z) & (z > thr)
                # only local maxima along frequency become seeds (speed; the neighbours of a
                # maximum are nearly always inside the extent it claims)
                zz = np.where(np.isfinite(z), z, -np.inf)
                ok &= (zz >= np.concatenate([[-np.inf], zz[:-1]])) & (zz >= np.concatenate([zz[1:], [-np.inf]]))
                for j in np.nonzero(ok)[0]:
                    lo, hi = a[j], a[j] + W
                    if self.fixed_mask[lo:hi].mean() > 0.5:
                        continue
                    wins.append((z[j], W, lo, slow))
        wins.sort(key=lambda w: -w[0])
        claimed = np.zeros(N, bool)
        claimed |= self.fixed_mask
        # DC / known spurs: claim their skirts so that they cannot start new signals
        Esm1 = np.nan_to_num(_movmean(Ef, 1))
        sg1 = self._sigma_any(sf, 1)
        for p in self.fixed_pos:
            if Esm1[p] > c.z_edge * sg1:
                s0 = self._extend(Esm1, np.zeros(N, bool), p, -1, c.z_edge * sg1, sg1)
                e0 = self._extend(Esm1, np.zeros(N, bool), p, +1, c.z_edge * sg1, sg1)
                claimed[s0:e0 + 1] = True
        self.spur_claim = claimed.copy()
        cands = []
        for z0, W, lo, slow in wins:
            hi = lo + W
            ctr = lo + (W - 1) / 2.0
            if claimed[int(round(ctr))]:
                continue
            E = Es if slow else Ef
            sd = ss if slow else sf
            free = ~claimed[lo:hi]
            if free.sum() < max(1, W // 2):
                continue
            zr = E[lo:hi][free].sum() / free.sum() / self._sigma_any(sd, free.sum())
            if zr < (c.z_det_slow if slow else c.z_det):
                continue
            # extent: on the slow spectrum when the emission is visible there
            # (FM spectra fluctuate strongly within one update), else on the fast one
            zsl = Es[lo:hi].mean() / self._sigma_any(ss, W)
            if zsl > c.z_edge * 2 or slow:
                Eext, sdx, sm = Es, ss, 1 if W <= 4 else 2
            else:
                Eext, sdx, sm = Ef, sf, max(1, W // 4)
            Esm = np.nan_to_num(_movmean(np.where(claimed, 0.0, Eext), sm))
            seg = Esm[lo:hi].copy()
            seg[claimed[lo:hi]] = -1e9
            p = lo + int(np.argmax(seg))
            pk = Esm[p]
            if pk <= 0:
                continue
            # a tail of an already claimed emission (monotonic towards it) is not a new signal
            if self._is_tail(np.nan_to_num(_movmean(Eext, sm)), claimed, p):
                continue
            sg_sm = self._sigma_any(sdx, sm)
            thr = max(c.z_edge * sg_sm, c.edge_frac * pk)
            s = self._extend(Esm, claimed, p, -1, thr, sg_sm)
            e = self._extend(Esm, claimed, p, +1, thr, sg_sm)
            ex = np.clip(Eext[s:e + 1], 0, None)
            if ex.sum() <= 0:
                continue
            idx = np.arange(s, e + 1)
            fc = float((idx * ex).sum() / ex.sum())
            wsum = e - s + 1
            zt = E[s:e + 1].mean() / self._sigma_any(sd, wsum)
            # 99 % power width of the excess, minus the kernel smear (quadrature)
            cs = np.cumsum(ex) / ex.sum()
            x0 = np.interp(0.005, cs, idx + 0.5) if len(idx) > 1 else s - 0.5
            x1 = np.interp(0.995, cs, idx + 0.5) if len(idx) > 1 else e + 0.5
            w99 = max(x1 - x0, 1.0)
            bw = math.sqrt(max(w99 ** 2 - self.smear ** 2, 0.25)) if w99 > 1 else 1.0
            cands.append(dict(s=int(s), e=int(e), fc=fc, bw=float(max(bw, 0.5)), z=float(max(zt, zr)),
                              snr_db=float(10 * math.log10(max(Eext[s:e + 1].mean(), 1e-6))),
                              slow=slow, parent=-1, p_ex=float(E[s:e + 1].mean())))
            claimed[s:e + 1] = True
        return cands

    def _is_tail(self, Esm, claimed, p):
        N = self.N
        for d in (-1, 1):
            j = p
            while 0 <= j + d < N and not claimed[j + d] and Esm[j + d] >= Esm[j] * 0.98:
                j += d
            if 0 <= j + d < N and claimed[j + d]:
                return True
            # peak right next to a claim
            if 0 <= p + d < N and claimed[p + d]:
                return True
        return False

    def _extend(self, Esm, claimed, p, d, thr, sig):
        """walk from the peak while above thr; stop at a significant valley."""
        N = self.N
        j = p
        run_max = Esm[p]
        vmin, vpos = Esm[p], p
        while 0 < j + d < N - 1 and not claimed[j + d]:
            v = Esm[j + d]
            if v < thr:
                break
            if v < vmin:
                vmin, vpos = v, j + d
            # rising again after a deep valley -> split at the valley
            if v > vmin and vpos != p and self.age_now[vpos] >= self.c.split_mature:
                rise = v - vmin
                if (vmin <= self.c.split_ratio * min(run_max, v) and rise > self.c.split_z * sig
                        and self._side_peak(Esm, j + d, d) > vmin / self.c.split_ratio):
                    return vpos - d if vpos != p else p
            if v > run_max and vpos == p:
                run_max = v
            j += d
        return j

    def _side_peak(self, Esm, j, d):
        N = self.N
        m = Esm[j]
        k = j
        while 0 <= k + d < N and Esm[k + d] >= m:
            k += d
            m = Esm[k]
        return m

    # ------------------------------------------------------------------ narrow in wide
    def _narrow(self, R, Rs, cands):
        """Narrow emissions inside wide ones. Analog FM (and most emissions) has a
        long-term spectrum symmetric about its carrier, so a narrow emission off the
        carrier shows up as a local asymmetry: E(c + d) >> E(c - d). The carrier c is
        the point of best symmetry. Features at the carrier itself cannot be separated
        (the IQ interval contains them anyway)."""
        c = self.c
        out = []
        sig1 = self._sigma_any(self.sig_s, 1)
        lim = 10 ** (c.narrow_db / 10)
        for cd in cands:
            if cd["parent"] != -1 or cd["e"] - cd["s"] + 1 < 5:
                continue
            s, e = cd["s"], cd["e"]
            L = np.log1p(np.clip(Rs - 1.0, -0.5, None))
            half = (e - s) // 2 + 1
            best, cbest = 1e18, cd["fc"]
            for cc in np.arange(cd["fc"] - 1.5, cd["fc"] + 1.51, 0.25):
                ds = np.arange(1, half + 1)
                xp, xm = cc + ds, cc - ds
                ok = (xp <= e + 1) & (xm >= s - 1) & (xp < self.N - 1) & (xm > 0)
                if ok.sum() < 3:
                    continue
                dif = np.abs(np.interp(xp[ok], np.arange(self.N), L) - np.interp(xm[ok], np.arange(self.N), L))
                dif = np.sort(dif)[:-2] if len(dif) > 4 else dif
                v = dif.mean()
                if v < best:
                    best, cbest = v, cc
            cd["fc_sym"] = float(cbest)
            # asymmetry map
            js = np.arange(s, e + 1)
            mir = 2 * cbest - js
            Em = np.interp(mir, np.arange(self.N), Rs, left=1.0, right=1.0)
            ratio = Rs[s:e + 1] / np.maximum(Em, 1e-3)
            zz = (Rs[s:e + 1] - Em) / (sig1 * math.sqrt(2.0))
            flag = (ratio > lim) & (zz > c.z_narrow) & (np.abs(js - cbest) >= 1.5)
            j = 0
            while j < len(flag):
                if flag[j]:
                    k = j
                    while k + 1 < len(flag) and flag[k + 1]:
                        k += 1
                    if k - j + 1 <= c.narrow_max_bins + 2:
                        idx = js[j:k + 1]
                        w = Rs[idx] - Em[j:k + 1]
                        fc = float((idx * w).sum() / w.sum())
                        bw = max(0.5, math.sqrt(max((k - j + 1) ** 2 - (self.smear * 0.6) ** 2, 0.25)))
                        out.append(dict(s=int(idx[0]), e=int(idx[-1]), fc=fc, bw=float(bw),
                                        z=float(zz[j:k + 1].max()),
                                        snr_db=float(10 * math.log10(max(w.mean(), 1e-6))),
                                        slow=1, parent=-2, p_ex=float(w.mean())))
                    j = k + 1
                else:
                    j += 1
        return out

    # ------------------------------------------------------------------ tracking
    def _track_stat(self, R, Rs, tr):
        s, e = max(0, tr.lo_bin), min(self.N - 1, tr.hi_bin)
        W = e - s + 1
        zf = (R[s:e + 1].mean() - 1) / self._sigma_any(self.sig_f, W)
        zs = (Rs[s:e + 1].mean() - 1) / self._sigma_any(self.sig_s, W)
        return max(zf, zs), float(R[s:e + 1].mean() - 1)

    def _track(self, R, Rs, cands):
        c = self.c
        live = [t for t in self.tracks if t.state != EXPIRED]
        used_c = set()
        matched = {}
        pairs = []
        for i, cd in enumerate(cands):
            for t in live:
                if (cd["parent"] == -2) != (t.parent >= 0):
                    continue
                d = abs(cd["fc"] - t.fc) / max(1.0, 0.5 * max(cd["bw"], t.bw))
                ov = min(cd["e"], t.hi_bin) - max(cd["s"], t.lo_bin)
                if d < 1.0 and ov >= 0:
                    pairs.append((d, i, t.id))
        pairs.sort()
        for d, i, tid in pairs:
            if i in used_c or tid in matched:
                continue
            used_c.add(i)
            matched[tid] = cands[i]
        # merged candidates also cover other tracks whose centre lies inside
        for i, cd in enumerate(cands):
            if cd["parent"] == -2:
                continue
            for t in live:
                if t.id not in matched and t.parent < 0 and cd["s"] <= t.fc <= cd["e"] and t.state in STREAMING:
                    matched[t.id] = dict(cd, merged_into=True)
        tau = 3.0
        for t in live:
            cd = matched.get(t.id)
            hit = False
            if cd is not None:
                hit = True
                if not cd.get("merged_into"):
                    t.fc += 0.3 * (cd["fc"] - t.fc)
                    dec = 1.0 - self.dt / tau
                    t.bw = min(max(cd["bw"], t.bw * dec), self.cap_bw)
                    t.ext = min(max(max(cd["fc"] - cd["s"], cd["e"] - cd["fc"]) + 0.5, t.ext * dec), self.cap_ext)
                    t.snr_db = cd["snr_db"]
                    t.z = cd["z"]
                    t.merged = False
                    others = [o for o in live if o is not t and o.parent < 0 and o.state in STREAMING
                              and cd["s"] <= o.fc <= cd["e"]]
                    if not others:
                        t.sep_hits += 1
                else:
                    t.merged = True
                t.lo_bin, t.hi_bin = self._extent(t)
            else:
                zk, _ = self._track_stat(R, Rs, t)
                t.z = zk
                if zk > c.z_keep and t.state != CAND:
                    hit = True
            _, pex = self._track_stat(R, Rs, t)
            t.phist = (t.phist + [pex])[-64:]
            t.fhist = (t.fhist + [t.fc])[-64:]
            t.bhist = (t.bhist + [t.bw])[-64:]
            t.hits = (t.hits + [1 if hit else 0])[-16:]
            if hit:
                t.nhits += 1
                t.t_last = self.t
            self._transition(t)
        # new tracks
        for i, cd in enumerate(cands):
            if i in used_c or cd.get("merged_into"):
                continue
            inside = [t for t in live if t.lo_bin <= cd["fc"] <= t.hi_bin and t.parent < 0]
            if cd["parent"] != -2 and inside:
                continue      # part of an existing (merged / wider) track
            if cd["parent"] != -2 and self._host_of_tail(cd["s"], cd["e"], cd["fc"], cd["snr_db"]) is not None:
                continue      # skirt of a stronger emission
            t = Track(id=self.next_id, fc=cd["fc"], bw=cd["bw"], snr_db=cd["snr_db"], z=cd["z"],
                      t_first=self.t, t_last=self.t, hits=[1], nhits=1)
            t.ext = min(max(cd["fc"] - cd["s"], cd["e"] - cd["fc"]) + 0.5, self.cap_ext)
            t.bw = min(t.bw, self.cap_bw)
            if cd["parent"] == -2:
                par = [p for p in live if p.parent < 0 and p.lo_bin <= cd["fc"] <= p.hi_bin]
                if not par:
                    continue
                t.parent = par[0].id
                t.independent = False
            t.lo_bin, t.hi_bin = self._extent(t)
            self.next_id += 1
            self._transition(t)
            self.tracks.append(t)
            live.append(t)
        # absorb young tracks that never showed up separately and sit inside an older one
        prim = [t for t in self.tracks if t.parent < 0 and t.state not in (EXPIRED, "DROP")]
        for t in prim:
            host = [o for o in prim if o is not t and abs(o.fc - t.fc) <= 64 and o.t_first <= t.t_first and o.state in STREAMING
                    and o.lo_bin <= t.fc <= o.hi_bin and o.state != "DROP"]
            if not host:
                ht = self._host_of_tail(t.lo_bin, t.hi_bin, t.fc, t.snr_db, exclude=t)
                if ht is not None and ht.t_first <= t.t_first:
                    host = [ht]
                    t.sep_hits = 0
            if host and t.sep_hits < 3:
                t.absorb += 1
                if t.absorb >= 2:
                    h = host[0]
                    lo, hi = min(h.lo_bin, t.lo_bin), max(h.hi_bin, t.hi_bin)
                    h.ext = min(max(h.ext, h.fc - lo + 0.5, hi - h.fc + 0.5), self.cap_ext)
                    h.bw = min(max(h.bw, (hi - lo + 1) - self.smear), self.cap_bw)
                    h.lo_bin, h.hi_bin = self._extent(h)
                    t.state = "DROP"
            else:
                t.absorb = 0
        # drop dead tracks
        self.tracks = [t for t in self.tracks if not (t.state == EXPIRED and self.t - t.t_last > 5.0)
                       and not (t.state == "DROP")]
        # child validation: independent power behaviour
        byid = {t.id: t for t in self.tracks}
        for t in self.tracks:
            if t.parent >= 0:
                p = byid.get(t.parent)
                if p is None or p.state == EXPIRED:
                    t.independent = True
                elif len(t.phist) >= 8:
                    a, b = np.array(t.phist[-16:]), np.array(p.phist[-16:])
                    n = min(len(a), len(b))
                    a, b = a[-n:], b[-n:]
                    if a.std() > 1e-9 and b.std() > 1e-9:
                        r = float(np.corrcoef(a, b)[0, 1])
                        if r < c.child_corr_max:
                            t.independent = True

    def _host_of_tail(self, s, e, fc, snr_db, exclude=None):
        """a streaming primary track that is >= 6 dB stronger, whose extraction interval
        contains fc, and towards which the emission's (slow) profile keeps rising:
        then [s, e] is a skirt of that emission, not a new one."""
        for h in self.tracks:
            if abs(h.fc - fc) > 64 or h is exclude or h.parent >= 0 or h.state not in STREAMING or h.snr_db < snr_db + 6.0:
                continue
            hs, he = self._extract(h)
            if not (hs <= fc <= he):
                continue
            seg = self.Rs[s:e + 1]
            if len(seg) == 0:
                continue
            pk = s + int(np.argmax(seg))
            if (h.fc < s and pk <= s + 1) or (h.fc > e and pk >= e - 1) or (s <= h.fc <= e):
                return h
        return None

    def _extract(self, t):
        c, bw = self.c, self.binw
        det_hz = t.bw * bw
        ext = max(det_hz * c.bw_factor, c.narrow_extract_hz if det_hz < 12000 else 0) + 2 * c.guard_hz
        half = ext / 2 / bw
        return max(2, int(math.floor(t.fc - half))), min(self.N - 3, int(math.ceil(t.fc + half)))

    def _extent(self, t):
        return int(math.floor(t.fc - t.ext + 0.5)), int(math.ceil(t.fc + t.ext - 0.5))

    def _transition(self, t):
        c = self.c
        recent = t.hits[-c.confirm_n:]
        if t.state == CAND:
            need = c.child_confirm if t.parent >= 0 else c.confirm_m
            if t.parent >= 0:
                ok = sum(t.hits[-need:]) >= need
            else:
                ok = sum(recent) >= c.confirm_m or t.z >= c.z_fast_confirm
            if ok:
                t.state = CONF
            elif len(t.hits) >= 2 and t.hits[-1] == 0 and t.hits[-2] == 0:
                t.state = "DROP"
            elif t.parent >= 0 and t.hits[-1] == 0:
                t.state = "DROP"
        elif t.state in (CONF, ACTIVE, HOLD):
            if t.hits[-1]:
                if t.state == HOLD or (t.state == CONF and t.nhits >= c.active_hits):
                    t.state = ACTIVE
            else:
                if self.t - t.t_last >= c.hold_s:
                    t.state = EXPIRED
                elif t.state != HOLD:
                    t.state = HOLD
        elif t.state == EXPIRED and t.hits[-1]:
            t.state = ACTIVE
        t.conf = min(1.0, t.nhits / 10.0) * (1.0 if t.state in (CONF, ACTIVE) else 0.5)

    # ------------------------------------------------------------------ allocation
    def streaming(self):
        c = self.c
        return [t for t in self.tracks if t.state in STREAMING or (c.stream_candidates and t.state == CAND)]

    def _allocate(self):
        c, N, bw = self.c, self.N, self.binw
        iv = []
        slots = []
        for t in self.streaming():
            s, e = self._extract(t)
            iv.append([s, e])
            if t.parent >= 0 and not t.independent:
                continue          # feature of its parent: no own slot (its bins are in the parent's)
            need = e - s + 1 + 2
            M = 8
            while M < need:
                M *= 2
            slots.append(dict(id=t.id, c=int(round(t.fc)), M=M, s=s, e=e, fc=t.fc, state=t.state))
        iv.sort()
        runs = []
        for s, e in iv:
            if runs and s <= runs[-1][1] + 1 + c.merge_gap_bins:
                runs[-1][1] = max(runs[-1][1], e)
            else:
                runs.append([s, e])
        return runs, slots
