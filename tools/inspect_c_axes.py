import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np
from scene2.perception import Detector,point_image,masks
s=dict(np.load(sys.argv[1]));p,v=point_image(s);m,_=masks(s['rgb']);b=Detector().detect(s,True)[1]['C']
roi=v&(p[:,:,2]>b.floor+.002)&(p[:,:,2]<b.rim+.12)
for j in (0,1):roi&=(p[:,:,j]>b.lower[j]-.025)&(p[:,:,j]<b.upper[j]+.025)
red=p[roi&m['C']];metal=p[roi&m['A']]
print('red',len(red),'metal',len(metal),'bin',b)
rng=np.random.default_rng(34);axes=[];left=metal.copy()
for _ in range(3):
 best=None
 for _ in range(1500):
  a,c=left[rng.choice(len(left),2,replace=False)];d=c-a;span=np.linalg.norm(d)
  if span<.04:continue
  d/=span;dist=np.linalg.norm(np.cross(left-a,d),axis=1);q=left[dist<.0028]
  if len(q)<8 or np.ptp(q@d)<.055:continue
  score=len(q)*np.ptp(q@d)
  if best is None or score>best[0]:best=(score,a,d,q)
 if best is None:break
 _,a,d,q=best;_,basis=np.linalg.eigh(np.cov(q.T));d=basis[:,-1];a=np.median(q,axis=0)
 axes.append((a,d));left=left[np.linalg.norm(np.cross(left-a,d),axis=1)>.0045]
 near=red[np.linalg.norm(np.cross(red-a,d),axis=1)<.018]
 print('axis',a,d,'support',len(q),'span',np.ptp(q@d),'near_red',len(near),'redspan',np.ptp(near@d) if len(near) else 0)
 if len(left)<8:break
if len(axes)>1:
 dist=np.column_stack([np.linalg.norm(np.cross(red-a,d),axis=1) for a,d in axes]);labels=dist.argmin(1)
 for i,(a,d) in enumerate(axes):
  q=red[(labels==i)&(dist[:,i]<.018)]
  print('assigned',i,len(q),'span',np.ptp(q@d) if len(q) else 0,'range',np.percentile(q@d,[1,99]) if len(q) else None)
 print('axis_separation',np.linalg.norm(np.cross(axes[1][0]-axes[0][0],axes[0][1])))
