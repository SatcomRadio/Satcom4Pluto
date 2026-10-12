import sys, time, numpy as np
from satref import scenarios as scn, evaluate as ev
from satref.det_v2 import Cfg
S=scn.suite(); names=sys.argv[1].split(','); K=sys.argv[2] if len(sys.argv)>2 else "sin5"
avg=256; upd=avg*(scn.N/2)/scn.FS
for nm in names:
    t0=time.time(); x,truth=scn.build(S[nm],2.5,seed=1)
    F,P5=ev.spectra(x,"sin5",avg); Pk=P5 if K=="sin5" else ev.spectra(x,K,avg)[1]
    r15=ev.score(truth,*ev.run_v15(P5,upd),upd)
    reps,masks,hist,d=ev.run_v2(Pk,upd,kernel=K)
    r2=ev.score(truth,reps,masks,upd)
    print("==",nm,"(%.1fs)"%(time.time()-t0))
    for name,r in (("v1.5",r15),("v2",r2)):
        print("  %-4s FA=%d (upd %d/%d) merges=%d"%(name,r['fa'],r['fa_updates'],r['upd'],r['merges']))
        for p in r['per']:
            print("     %-4s f=%8.0f snr=%5.1f (+%.1fdB) b99=%5.0f Pd=%.2f lat=%.3f ferr=%5.0f berr=%6.0f split=%.2f"%(p['cls'],p['f'],p['snr'],p['above'],p['b99'],p['pd'],p['latency'],p['ferr'],p['berr'],p['split_frac']))
    print("  v2 tracks:",[(t.id,t.state,round((t.fc-scn.N/2)*scn.BINW),round(t.bw*scn.BINW),t.parent) for t in d.tracks])
