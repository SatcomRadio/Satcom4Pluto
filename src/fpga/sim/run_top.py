import numpy as np, subprocess, sys, os
L=int(os.environ.get('L','8')); N=1<<L; H=N//2
NG=6; SC=L-NG                      # scale stages
NHOP=int(os.environ.get('NHOP','40'))
rng=np.random.default_rng(7)
ns=(NHOP+2)*H
t=np.arange(ns)
x=rng.normal(0,4,ns)+1j*rng.normal(0,4,ns)
tones=[(20.3,300),(60.0,40),(-45.5,1500),(100.25,10)]
for f,a in tones: x+=a*np.exp(2j*np.pi*f*t/N)
xi=np.clip(np.round(x.real),-2048,2047).astype(int); xq=np.clip(np.round(x.imag),-2048,2047).astype(int)
with open('in.txt','w') as f:
    f.write('%d\n'%ns)
    for a,b in zip(xi,xq): f.write('%d %d\n'%(a,b))
# gains: runs in b-order (b<N/2 positive freq)
gain=np.zeros(N,int)
runs=[(0,3),(15,11),(55,11),(N-52,14)]
for s,l in runs: gain[s:s+l]=255
gain[55:60]=128
gw=[sum(int(gain[4*a+k])<<(8*k) for k in range(4)) for a in range(N//4)]
KERN=int(os.environ.get('KERN','0')); AVG=2; OSH=int(os.environ.get("OSH","6")); PSH=int(os.environ.get("PSH","0"))
cmd=[]
cmd.append('w 0014 %08x'%(AVG|(PSH<<8)|(OSH<<16)|(KERN<<24)))
for a,w in enumerate(gw): cmd.append('w %04x %08x'%(0x4000+4*a,w))
for r,(s,l) in enumerate(runs): cmd.append('w %04x %08x'%(0x6000+4*r,(l<<16)|s))
cmd.append('w 0034 %08x'%len(runs))
cmd.append('w 0030 00000001')
cmd.append('w 0010 00000007')
cmd.append('waitspec')
cmd.append('w 001c 00000001')
cmd.append('waitspec')
cmd.append('wait %d'%int(ns*1.63*2*0.95 - 0))
cmd.append('status')
cmd.append('end')
open('cmd.txt','w').write('\n'.join(cmd)+'\n')
srcs=['tb_top.v','../rtl/sgc_top.v','../rtl/sgc_core.v','../rtl/sgc_fft.v','../rtl/sgc_afifo.v']
subprocess.run(['iverilog','-g2005','-o','tbt','-Ptb_top.L=%d'%L]+srcs,check=True)
r=subprocess.run(['vvp','-n','tbt'],check=True,capture_output=True,text=True)
print(r.stdout[-1500:])
# ---------- parse output
w=[int(s,16) for s in open('out.txt').read().split()]
def s16(v): return v-65536 if v>=32768 else v
frames={}
i=0; bad=0
while i<len(w):
    if w[i]!=0x53474348: i+=1; bad+=1; continue
    if i+3>len(w): break
    seq=w[i+1]; h2=w[i+2]; nr=h2&0xFFFF; osh=h2>>28; ll=(h2>>24)&15
    j=i+3; X=np.zeros(N,complex); ok=True
    for r_ in range(nr):
        if j>=len(w): ok=False;break
        s=w[j]&0xFFFF; l=w[j]>>16; j+=1
        if j+l>len(w): ok=False;break
        for q in range(l):
            X[s+q]=s16(w[j+q]&0xFFFF)+1j*s16(w[j+q]>>16)
        j+=l
    if not ok: break
    frames[seq]=(X*2**osh, h2)
    i=j
print('frames',len(frames),'skipped words',bad,'seqs',sorted(frames)[:3],'...',sorted(frames)[-2:])
# ---------- reference
n=np.arange(N)
wv=np.floor(np.sin(np.pi*n/N)*131072+0.5); wv=np.minimum(wv,131071)
OFF=0
def ref(k):
    o=OFF+k*H
    seg=(xi[o:o+N]+1j*xq[o:o+N])
    xw=np.floor((seg.real*wv)/2**13+0.5)+1j*np.floor((seg.imag*wv)/2**13+0.5)
    return np.fft.fft(xw)/2**SC
kk0=sorted(frames)[3]; m0=gain==255
OFF=min(range(0,4*H),key=lambda o:(globals().__setitem__('OFF',o),np.max(np.abs(frames[kk0][0][m0]-ref(kk0)[m0])))[1])
print('input offset of sequence 0:',OFF)
errs=[]
for k,(X,h2) in sorted(frames.items()):
    if OFF+k*H+N>ns: continue
    R=ref(k)*gain/256.0
    R[gain==255]=ref(k)[gain==255]
    m=gain>0
    e=np.max(np.abs(X[m]-R[m])); errs.append(e)
print('max |fpga - ref| over selected bins per frame: median %.2f max %.2f (bin rms %.1f)'%(np.median(errs),np.max(errs),np.sqrt(np.mean(np.abs(R[m])**2))))
# ---------- spectrum
sp=open('spec.txt').read().split('#')
S=[np.array([int(v) for v in s.split()]) for s in sp if s.strip()]
print('spectra read',len(S))
def kern(X):
    if KERN==0: return (np.roll(X,2)-4*np.roll(X,1)+6*X-4*np.roll(X,-1)+np.roll(X,-2))/16
    if KERN==1: return (-np.roll(X,1)+2*X-np.roll(X,-1))/4
    return X
P=[np.abs(np.round(kern(ref(k)).real)+1j*np.round(kern(ref(k)).imag))**2/2**PSH for k in range(NHOP-3)]
best=None
for k0 in range(len(P)-(1<<AVG)):
    Pr=sum(P[k0:k0+(1<<AVG)])
    m=np.arange(3,N-3)
    err=np.median(np.abs(S[-1][m]-Pr[m])/(Pr[m]+50))
    if best is None or err<best[0]: best=(err,k0)
print('spectrum match: median rel err %.3f at start frame %d'%best)
Pr=sum(P[best[1]:best[1]+(1<<AVG)])
for b in [20,60,N-45,100,5,128 if N>128 else 7]:
    print('  bin %d fpga %d ref %.0f'%(b,S[-1][b],Pr[b]))
# alignment check
for k in sorted(frames)[3:6]:
    X=frames[k][0]; m=gain==255
    es=[(np.max(np.abs(X[m]-ref(kk)[m])),kk) for kk in range(max(0,k-2),min(NHOP,k+3))]
    print('frame seq',k,'errors vs ref frames',['%d:%.1f'%(kk,e) for e,kk in es])
