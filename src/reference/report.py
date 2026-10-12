import json,sys,numpy as np
res=json.load(open(sys.argv[1]))
print("%-22s | %-46s | %-46s"%("scenario","v1.5: Pd  lat   FA/s merg ferr  |berr| rate","v2: Pd  lat   FA/s merg ferr  |berr| rate"))
for r in res:
    row="%-22s"%r['name']
    for k in ('v15','v2'):
        x=r[k]; per=x['per']
        if per:
            pd=np.nanmean([p['pd'] for p in per]); lat=np.nanmean([p['latency'] for p in per])
            fe=np.nanmedian([p['ferr'] for p in per]); be=np.nanmedian([abs(p['berr']) for p in per])
        else: pd=lat=fe=be=float('nan')
        row+=" | %4.2f %5.3f %5.2f %4d %5.0f %6.0f %5.2fM"%(pd,lat,x['fa_per_s'],x['merges'],fe,be,x['rate']/1e6)
    print(row)
