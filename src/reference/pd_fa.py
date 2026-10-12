"""Detection probability vs SNR and false-alarm rate on long noise-only runs.

usage: python3 pd_fa.py fa SECONDS KERNEL      -> false alarms (white floor and rippled floor)
       python3 pd_fa.py pd KERNEL               -> Pd / latency vs in-band SNR
"""
import sys, json, math, time, numpy as np, multiprocessing as mp
from satref import scenarios as scn, evaluate as ev, fpga_model as fm, signals as sg
from satref.det_v15 import DetV15
from satref.det_v2 import DetV2, Cfg, STREAMING

AVG = 256
UPD = AVG * (scn.N / 2) / scn.FS


def fa_run(args):
    secs, kernel, ripple, seed = args
    chunk = 2.0
    d15 = DetV15(scn.N, scn.BINW, UPD)
    d2 = DetV2(scn.N, scn.BINW, UPD, Cfg(), kernel=kernel)
    floor = scn._ripple(1.0, 3.0, 0.9) if ripple else None
    conf_ids = set()
    fa15_upd = 0
    fa15_runs = 0
    open15 = 0
    open2 = 0
    nupd = 0
    t = 0.0
    tail = None
    for c in range(int(secs / chunk)):
        x = sg.shaped_noise(int(chunk * scn.FS), scn.FS, floor, seed=seed * 1000 + c)
        if tail is not None:
            x = np.concatenate([tail, x])
        F = fm.analysis(x.astype(np.complex64), scn.N)
        used = F.shape[0] * (scn.N // 2)
        tail = x[used:]
        F15 = fm.centred(fm.det_spectrum(F, "sin5", AVG))
        Fk = F15 if kernel == "sin5" else fm.centred(fm.det_spectrum(F, kernel, AVG))
        for u in range(F15.shape[0]):
            t += UPD
            dets, op = d15.update(F15[u])
            fa15_runs += len(dets)
            fa15_upd += len(dets) > 0
            open15 += op.sum()
            tracks, runs, slots = d2.update(Fk[u], t=t)
            for tr in tracks:
                if tr.state in STREAMING:
                    conf_ids.add(tr.id)
            open2 += sum(e - s + 1 for s, e in runs)
            nupd += 1
    return dict(secs=nupd * UPD, ripple=ripple, v15_runs=fa15_runs, v15_upd=fa15_upd,
                v15_open_frac=open15 / nupd / scn.N, v2_confirmed=len(conf_ids), v2_open_frac=open2 / nupd / scn.N)


def pd_run(args):
    cls, snr, kernel, seed = args
    dur = 3.0
    f0 = [-1700e3, -900e3, -100e3, 700e3, 1500e3]
    sc = dict(emitters=[scn.emitter(cls, f, snr, seed=seed * 10 + i, on=[(0.5, dur)]) for i, f in enumerate(f0)])
    x, truth = scn.build(sc, dur, seed=seed)
    F, P5 = ev.spectra(x, "sin5", AVG)
    Pk = P5 if kernel == "sin5" else ev.spectra(x, kernel, AVG)[1]
    r15 = ev.score(truth, *ev.run_v15(P5, UPD), UPD)
    reps, masks, _, _ = ev.run_v2(Pk, UPD, kernel=kernel)
    r2 = ev.score(truth, reps, masks, UPD)
    out = dict(cls=cls, snr=snr, above=truth[0]["above_floor_db"], seed=seed)
    for k, r in (("v15", r15), ("v2", r2)):
        out[k + "_pd"] = float(np.nanmean([p["pd"] for p in r["per"]]))
        out[k + "_lat"] = float(np.nanmedian([p["latency"] for p in r["per"]]))
        out[k + "_det"] = float(np.mean([np.isfinite(p["latency"]) for p in r["per"]]))
    return out


if __name__ == "__main__":
    mode = sys.argv[1]
    t0 = time.time()
    if mode == "fa":
        secs = float(sys.argv[2])
        kernel = sys.argv[3]
        jobs = [(secs, kernel, False, 1), (secs, kernel, True, 2)]
        with mp.Pool(2) as p:
            res = p.map(fa_run, jobs)
        json.dump(res, open(f"fa_{kernel}.json", "w"), indent=1)
        for r in res:
            print(r)
    else:
        kernel = sys.argv[2]
        jobs = [(c, s, kernel, sd) for c in ("N5", "N7", "W40") for s in (-8, -6, -4, -2, 0, 2) for sd in (1,)]
        with mp.Pool(2) as p:
            res = p.map(pd_run, jobs)
        json.dump(res, open(f"pd_{kernel}.json", "w"), indent=1)
        print("cls  snr  above | v1.5 Pd  det  lat  | v2 Pd  det  lat")
        for r in res:
            print("%-4s %4.0f %5.1f | %5.2f %4.2f %5.2f | %5.2f %4.2f %5.2f" % (
                r["cls"], r["snr"], r["above"], r["v15_pd"], r["v15_det"], r["v15_lat"], r["v2_pd"], r["v2_det"], r["v2_lat"]))
    print("%.0f s" % (time.time() - t0))
