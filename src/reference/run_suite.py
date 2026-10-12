"""Run the scenario suite: v1.5 (sin5, as in SGCH v1.0) vs v2 (kernel selectable)."""
import sys, json, time, numpy as np, multiprocessing as mp
from satref import scenarios as scn, evaluate as ev
from satref.det_v2 import Cfg
AVG=256; UPD=AVG*(scn.N/2)/scn.FS; DUR=3.0
def one(args):
    nm,kernel,seed=args
    S=scn.suite(); sc=S[nm]
    x,truth=scn.build(sc,DUR,seed=seed)
    F,P5=ev.spectra(x,"sin5",AVG)
    Pk=P5 if kernel=="sin5" else ev.spectra(x,kernel,AVG)[1]
    ign=[f for f,a in sc.get("spurs",[])]+([0.0] if sc.get("dc") else [])
    r15=ev.score(truth,*ev.run_v15(P5,UPD),UPD,ignore=ign)
    m15=ev.run_v15(P5,UPD)[1]
    cfg=Cfg(); 
    if sc.get("spurs"): cfg.spur_hz=tuple(f for f,a in sc["spurs"])
    reps,masks,hist,d=ev.run_v2(Pk,UPD,cfg=cfg,kernel=kernel)
    r2=ev.score(truth,reps,masks,UPD,ignore=ign)
    for r,m in ((r15,m15),(r2,masks)):
        r["rate"]=ev.out_rate(m,UPD); r["fa_per_s"]=r["fa"]/(len(m)*UPD)
    return dict(name=nm,seed=seed,kernel=kernel,v15=r15,v2=r2,dur=len(masks)*UPD)
if __name__=="__main__":
    kernel=sys.argv[1] if len(sys.argv)>1 else "sin3"
    seeds=[int(s) for s in (sys.argv[2].split(',') if len(sys.argv)>2 else ["1"])]
    names=sys.argv[3].split(',') if len(sys.argv)>3 else list(scn.suite().keys())
    jobs=[(n,kernel,s) for n in names for s in seeds]
    t0=time.time()
    with mp.Pool(min(len(jobs),mp.cpu_count())) as p: res=p.map(one,jobs)
    out=sys.argv[4] if len(sys.argv)>4 else f"results_{kernel}.json"
    json.dump(res,open(out,"w"),indent=1,default=float)
    print("done %.0fs -> %s"%(time.time()-t0,out))
