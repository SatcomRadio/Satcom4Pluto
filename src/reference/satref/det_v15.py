"""Port of the SGCH v1.0 / sat_gate v1.5 detector (sgchd.c detect_update, det mode).

Input: centred spectrum P (linear). Produces per update the candidate runs and the
open-bin mask (with hang/hold). Gains/taper/release are not modelled except the
open mask, which is what decides the streamed bins.
"""
import numpy as np


class DetV15:
    def __init__(self, N, binw, update_s, min_snr_db=3.0, res_bw=5.0, display_bw=40.0,
                 merge_khz=None, hold_s=1.5, dc_notch_khz=5.0, seg_l=8):
        self.N, self.binw = N, binw
        self.kch = int(min(64, max(1, round(res_bw * 1e3 / binw))))
        self.khalf = (self.kch - 1) // 2
        self.dhalf = max(1, int(round(display_bw * 1e3 / 2 / binw)))
        mk = res_bw if merge_khz is None else merge_khz
        self.gap = max(1, int(round(mk * 1e3 / binw)))
        self.hang_n = int(min(255, max(1, np.ceil(hold_s / update_s) + 1)))
        self.tmin = min_snr_db
        self.dc_notch = dc_notch_khz * 1e3
        self.seg = 1 << seg_l
        self.hang = np.zeros(N, int)

    def update(self, P):
        N, kch = self.N, self.kch
        # channel sums (window of kch bins starting at j - khalf)
        c = np.concatenate([[0.0], np.cumsum(P)])
        a = np.arange(N) - self.khalf
        b = a + kch
        S = c[np.clip(b, 0, N)] - c[np.clip(a, 0, N)]
        lev = 10 * np.log10(np.maximum(S, 1e-30))
        nseg = N // self.seg
        segfl = np.array([np.sort(lev[s * self.seg:(s + 1) * self.seg])[self.seg // 4] for s in range(nseg)])
        flo = np.empty(N)
        for j in range(N):
            sg = j // self.seg
            flo[j] = segfl[max(0, sg - 2):min(nseg, sg + 3)].min()
        cand = []
        in_run = False
        for j in range(N):
            cc = (lev[j] >= flo[j] + self.tmin) and 2 < j < N - 3 and abs(j - N // 2) * self.binw > self.dc_notch
            if cc:
                if not in_run:
                    in_run = True
                    r = dict(s=j, e=j, ppos=j, peak=lev[j], floor=flo[j])
                else:
                    r["e"] = j
                    if lev[j] > r["peak"]:
                        r["peak"], r["ppos"] = lev[j], j
                    r["floor"] = min(r["floor"], flo[j])
            if in_run and ((not cc and j - r["e"] > self.gap) or j == N - 1):
                in_run = False
                if r["e"] - r["s"] + 1 >= kch:
                    cand.append(r)
        hit = np.zeros(N, bool)
        for r in cand:
            hit[max(0, r["ppos"] - self.dhalf):min(N, r["ppos"] + self.dhalf + 1)] = True
        self.hang[hit] = self.hang_n
        self.hang[~hit] = np.maximum(self.hang[~hit] - 1, 0)
        open_ = hit | (self.hang > 1)
        open_[[0, 1, N - 2, N - 1]] = False
        dets = [dict(fc=r["ppos"], s=r["s"], e=r["e"], snr_db=r["peak"] - r["floor"]) for r in cand]
        return dets, open_
