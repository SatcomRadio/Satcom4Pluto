"""FM audio through the whole chain: FPGA analysis -> detector -> selected bins ->
(v2) per-signal decimated slot synthesis (what sgch_rx serves to SDRangel) or
(v1.5) wideband rebuild + DDC -> FM demodulator -> compare with the known audio.

usage: python3 audio_test.py [KERNEL]
"""
import sys, json, math, numpy as np
from satref import scenarios as scn, evaluate as ev, fpga_model as fm, fmdemod as dm
from satref.det_v15 import DetV15
from satref.det_v2 import DetV2, Cfg, STREAMING

AVG = 256
UPD = AVG * (scn.N / 2) / scn.FS
DUR = 3.0
KERNEL = sys.argv[1] if len(sys.argv) > 1 else "sin3"

SC = {
    "clean": dict(emitters=[scn.emitter("N5", -1203e3, 20), scn.emitter("N7", -402e3, 20, seed=2),
                            scn.emitter("W25", 398e3, 20, seed=3), scn.emitter("W40", 1201e3, 20, seed=4)]),
    "moderate": dict(emitters=[scn.emitter("N5", -1203e3, 10), scn.emitter("N7", -402e3, 10, seed=2),
                               scn.emitter("W25", 398e3, 12, seed=3), scn.emitter("W40", 1201e3, 12, seed=4)]),
    "adjacent": dict(emitters=[scn.emitter("W40", 300e3, 15), scn.emitter("W40", 350e3, 15, seed=2),
                               scn.emitter("N7", -500e3, 15, seed=3), scn.emitter("N7", -487e3, 12, seed=4),
                               scn.emitter("N5", 900e3, 15, seed=5), scn.emitter("W40", 945e3, 35, seed=6)]),
    "bursty": dict(emitters=[scn.emitter("N5", -700e3, 15, on=[(0.3, 1.2), (1.5, 2.9)]),
                             scn.emitter("W40", 600e3, 15, fade=10, seed=2)]),
}


def run(name):
    sc = SC[name]
    x, truth = scn.build(sc, DUR, seed=11)
    F = fm.analysis(x.astype(np.complex64), scn.N)
    P5 = fm.centred(fm.det_spectrum(F, "sin5", AVG))
    Pk = P5 if KERNEL == "sin5" else fm.centred(fm.det_spectrum(F, KERNEL, AVG))
    nfr = F.shape[0]
    d2 = DetV2(scn.N, scn.BINW, UPD, Cfg(), kernel=KERNEL)
    d15 = DetV15(scn.N, scn.BINW, UPD)
    # per-frame masks (centred bins); decisions of update u apply from update u+1
    sel2 = np.zeros((nfr, scn.N), bool)
    sel15 = np.zeros((nfr, scn.N), bool)
    slots = {}            # track id -> dict(c, M, ext per frame)
    ext = {}
    for u in range(P5.shape[0]):
        tracks, runs, sl = d2.update(Pk[u], t=(u + 1) * UPD)
        _, op = d15.update(P5[u])
        f0, f1 = (u + 1) * AVG, min(nfr, (u + 2) * AVG)
        if f0 >= nfr:
            break
        for s, e in runs:
            sel2[f0:f1, s:e + 1] = True
        sel15[f0:f1] = op[None, :]
        for s in sl:
            if s["id"] not in slots:
                slots[s["id"]] = dict(c=s["c"], M=s["M"], first=f0, recentre=0)
            o = slots[s["id"]]
            if s["s"] < o["c"] - o["M"] // 2 + 1 or s["e"] > o["c"] + o["M"] // 2 - 2:
                o["recentre"] += 1
            ext.setdefault(s["id"], []).append((f0, f1, s["s"], s["e"]))
    words2 = sel2.sum() / nfr * (scn.FS / (scn.N / 2))
    words15 = sel15.sum() / nfr * (scn.FS / (scn.N / 2))
    out = []
    G15 = np.fft.ifftshift(sel15, axes=1).astype(float)
    y15 = fm.synth_full(F, G15)
    for tr in truth:
        fc_bin = scn.N / 2 + tr["f"] / scn.BINW
        bwx = 2 * (tr["dev"] + 3000) * 1.1            # Carson + 10 %
        fs_ref = 48000 if tr["dev"] < 5000 else 96000
        # ideal: DDC of the unprocessed input
        a_ref = dm.audio_from_iq(dm.ddc(x, scn.FS, tr["f"], bwx, fs_ref), fs_ref, tr["dev"])
        s_ref = dm.audio_snr(a_ref, tr["audio"], 0.5)
        # v1.5: wideband rebuild of the open bins + DDC
        a15 = dm.audio_from_iq(dm.ddc(y15[:len(x)], scn.FS, tr["f"], bwx, fs_ref), fs_ref, tr["dev"])
        s15 = dm.audio_snr(a15, tr["audio"], 0.5)
        # v2: the slot of the track that covers this emission
        best = None
        for tid, o in slots.items():
            e_ = ext[tid][-1]
            if e_[2] <= fc_bin <= e_[3]:
                # prefer the slot that lives longest (absorbed tracks lose their slot), then the closest
                key = (-e_[1], abs(o["c"] - fc_bin))
                if best is None or key < best[0]:
                    best = (key, tid)
        if best is None:
            out.append(dict(name=name, cls=tr["cls"], snr=tr["snr_db"], ref=s_ref[0], v15=s15[0], v15_bad=s15[1],
                            v2=float("nan"), v2_bad=1.0, slot_rate=0))
            continue
        tid = best[1]
        o = slots[tid]
        # PC mask: bins inside the track's extraction interval and actually received
        msk = np.zeros((nfr, o["M"]))
        d = np.arange(-o["M"] // 2, o["M"] // 2)
        for f0, f1, s, e in ext[tid]:
            inb = (o["c"] + d >= s) & (o["c"] + d <= e)
            msk[f0:f1] = inb[None, :]
        msk *= sel2[:, (o["c"] + d) % scn.N]
        c_nat = o["c"] - scn.N // 2
        D = scn.N // o["M"]
        # synth_slot with a per-frame mask
        Zs = F[:, (c_nat + d) % scn.N] * msk
        Ff = np.zeros((nfr, scn.N), np.complex64)
        Ff[:, (c_nat + d) % scn.N] = Zs
        y = fm.synth_slot(Ff, c_nat, o["M"])
        fs_slot = scn.FS * o["M"] / scn.N
        # the downlink sits at (fc_bin - c) bins from the slot centre; SDRangel's channel
        # is tuned there -- here an exact mix + the same demodulator
        off = (fc_bin - o["c"]) * scn.BINW
        n = np.arange(len(y))
        yb = y * np.exp(-2j * np.pi * off * n / fs_slot)
        from scipy import signal as sps
        h = sps.firwin(255, bwx / 2, fs=fs_slot) if bwx < fs_slot else None
        if h is not None:
            yb = sps.oaconvolve(yb, h, mode="same")
        a2 = dm.audio_from_iq(yb, fs_slot, tr["dev"])
        s2 = dm.audio_snr(a2, tr["audio"], 0.5 + (o["first"] * scn.N / 2) / scn.FS)
        out.append(dict(name=name, cls=tr["cls"], snr=tr["snr_db"], ref=s_ref[0], v15=s15[0], v15_bad=s15[1],
                        v2=s2[0], v2_bad=s2[1], slot_rate=fs_slot, M=o["M"], recentre=o["recentre"],
                        slot_start_s=o["first"] * (scn.N / 2) / scn.FS))
    return out, words15, words2


if __name__ == "__main__":
    allr = []
    print("scenario  cls  SNR | ideal DDC | v1.5 rebuild (bad 20ms) | v2 slot (bad) rate  start")
    for nm in SC:
        res, w15, w2 = run(nm)
        for r in res:
            print("%-9s %-4s %4.0f | %6.1f dB | %6.1f dB (%4.2f) | %6.1f dB (%4.2f) %6.0f %5.2f" % (
                nm, r["cls"], r["snr"], r["ref"], r["v15"], r["v15_bad"], r["v2"], r["v2_bad"], r["slot_rate"],
                r.get("slot_start_s", float("nan"))))
        print("   link words/s: v1.5 %.0f  v2 %.0f" % (w15, w2))
        allr += res
    json.dump(allr, open(f"audio_{KERNEL}.json", "w"), indent=1, default=float)
