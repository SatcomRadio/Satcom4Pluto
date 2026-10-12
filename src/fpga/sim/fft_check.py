import numpy as np, subprocess, sys
L=int(sys.argv[1]); NG=int(sys.argv[2]); N=1<<L; NF=4
rng=np.random.default_rng(1)
def br(v):
    return int(format(v,'0%db'%L)[::-1],2)
X=[]
with open('fft_in.txt','w') as f:
    f.write('%d\n'%NF)
    for fr in range(NF):
        t=np.arange(N)
        x=(rng.normal(0,3000,N)+1j*rng.normal(0,3000,N)) + 8000*np.exp(2j*np.pi*(5.3+fr)*t/N)
        x=np.round(x.real)+1j*np.round(x.imag)
        x=np.clip(x.real,-32768,32767)+1j*np.clip(x.imag,-32768,32767)
        X.append(np.fft.fft(x)/2**(L-NG))
        for p in range(N):
            v=x[br(p)]; f.write('%d %d\n'%(v.real,v.imag))
subprocess.run(['iverilog','-g2005','-o','tbf','-Ptb_fft.L=%d'%L,'-Ptb_fft.NG=%d'%NG,'tb_fft.v','../rtl/sgc_fft.v'],check=True)
subprocess.run(['vvp','-n','tbf'],check=True,stdout=subprocess.DEVNULL)
o=np.loadtxt('fft_out.txt',dtype=np.int64)
# find frames: sequences where idx goes 0..N-1
idx=o[:,2]; starts=[i for i in range(len(idx)-N+1) if idx[i]==0 and idx[i+N-1]==N-1 and np.all(idx[i:i+N]==np.arange(N))]
print('frames found',len(starts))
for fr,s in enumerate(starts[:NF]):
    best=None
    y=o[s:s+N,0]+1j*o[s:s+N,1]
    errs=[np.max(np.abs(y-Xr)) for Xr in X]
    j=int(np.argmin(errs)); rel=np.sqrt(np.mean(np.abs(y-X[j])**2))/np.sqrt(np.mean(np.abs(X[j])**2))
    print('out frame',fr,'matches input frame',j,'max err %.2f'%errs[j],'rel rms err %.2e'%rel)
