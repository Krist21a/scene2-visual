import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
from scene2.perception import *
s=dict(np.load(sys.argv[1]));points,valid=point_image(s);_,bins=Detector().detect(s)
hsv=cv2.cvtColor(s['rgb'],cv2.COLOR_RGB2HSV)
b=bins[sys.argv[2]]
region=valid&(hsv[:,:,1]<75)&(hsv[:,:,2]>40)&(points[:,:,2]>b.floor+.003)&(points[:,:,2]<b.rim+.02)
for j in (0,1):region&=(points[:,:,j]>b.lower[j]-.02)&(points[:,:,j]<b.upper[j]+.02)
for mask in components(cv2.morphologyEx(region.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8)),10):
 mask&=region;p=points[mask]
 _,basis=np.linalg.eigh(np.cov(p.T));lo,hi=np.percentile(p@basis,[1,99],axis=0)
 print(json.dumps(dict(n=len(p),pca_extent=(hi-lo).tolist(),bounds=[p.min(0).tolist(),p.max(0).tolist()],white=int((hsv[mask,2]>125).sum()))))
cv2.imwrite('/tmp/bin-shape.png',region.astype(np.uint8)*255)
