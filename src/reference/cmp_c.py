"""Compare the C detector (sw/arm/det2.c via det2_test) with det_v2.py on the same spectra."""
import sys, struct, subprocess, numpy as np
from satref import scenarios as scn, evaluate as ev
from satref.det_v2 import DetV2, Cfg
AVG=256; UPD=AVG*(scn.N/2)/scn.FS
KMAP={"sin5":0,"sin3":1,"sin1":2}
def run(nm,kernel="sin3",dur=2.0):
    x,truth=scn.build(scn.suite()[nm],dur,seed=3)
    F,P=ev.spectra(x,kernel,AVG)
    with open("/tmp/spec.bin","wb") as f:
        f.write(struct.pack("<iii",scn.N,P.shape[0],KMAP[kernel])); f.write(struct.pack("<dd",scn.BINW,UPD))
        f.write(P.astype("<f8").tobytes())
    out=subprocess.run(["../sw/arm/det2_test","/tmp/spec.bin"],capture_output=True,text=True,check=True).stdout
    cu=[];
    for line in out.splitlines():
        p=line.split()
        if p[0]=="U": cu.append(dict(T=[],R=[]))
        elif p[0]=="T": cu[-1]["T"].append((int(p[1]),p[2],float(p[3]),float(p[4]),int(p[5]),int(p[6])))
        elif p[0]=="R": cu[-1]["R"].append((int(p[1]),int(p[2])))
    d=DetV2(scn.N,scn.BINW,UPD,Cfg(),kernel=kernel)
    nd=0; maxdf=0; nr=0
    for u in range(P.shape[0]):
        tr,runs,sl=d.update(P[u],t=(u+1)*UPD)
        py=[(t.id,t.state,t.fc,t.bw,t.lo_bin,t.hi_bin) for t in tr]
        c=cu[u]["T"]
        if [(a[0],a[1]) for a in py]!=[(a[0],a[1]) for a in c]:
            nd+=1
            if nd<=3: print("  upd",u,"py",[(a[0],a[1][:4],round(a[2],2)) for a in py],"\n         c ",[(a[0],a[1][:4],round(a[2],2)) for a in c])
        else:
            for a,b in zip(py,c): maxdf=max(maxdf,abs(a[2]-b[2]),abs(a[3]-b[3]))
        if [tuple(r) for r in runs]!=cu[u]["R"]: nr+=1
    print("%-18s updates %d: track-list mismatches %d, run mismatches %d, max |fc|,|bw| diff %.2e bins"%(nm,P.shape[0],nd,nr,maxdf))
for nm in sys.argv[1].split(","): run(nm, sys.argv[2] if len(sys.argv)>2 else "sin3")
