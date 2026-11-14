"""Derive conservative open-finger slabs from the public 2F85 collision mesh.
No scene, object state, or joint telemetry is read.
"""
import sys,struct,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
root=Path(__file__).resolve().parents[1];path=root/'assets/follower.stl';raw=path.read_bytes();n=struct.unpack_from('<I',raw,80)[0]
v=np.array([struct.unpack_from('<3f',raw,84+50*i+12+12*j) for i in range(n) for j in range(3)])*.001
# Public zero-angle follower transform, expressed about the pinch/TCP site.
v=np.column_stack((.0682+v[:,1],v[:,0],.0466-v[:,2]));v=np.unique(v,axis=0)
hull=ConvexHull(v);edges=set()
for face in hull.simplices:
 for a,b in ((face[0],face[1]),(face[1],face[2]),(face[2],face[0])):edges.add(tuple(sorted((a,b))))
boxes=[]
for low,high in zip([-.005,.005,.015,.025,.04],[.005,.015,.025,.04,.055]):
 points=list(v[(v[:,2]>=low)&(v[:,2]<=high)])
 for a,b in edges:
  u,w=v[a],v[b]
  for height in (low,high):
   if abs(w[2]-u[2])<1e-12:continue
   t=(height-u[2])/(w[2]-u[2])
   if 0<=t<=1:points.append(u+t*(w-u))
 if not points:continue
 points=np.array(points);lo=np.floor(points.min(0)*1e6)/1e6;hi=np.ceil(points.max(0)*1e6)/1e6
 boxes.append(dict(center=((lo+hi)/2).tolist(),half=((hi-lo)/2).tolist(),source='follower_convex_hull_slab'))
for z in (.004955,.023705):boxes.append(dict(center=[.0467,0,z],half=[.004,.011,.009375],source='public_pad_box'))
out=dict(source_mesh_sha256=hashlib.sha256(raw).hexdigest(),units='metres',configuration='open_driver_zero',positive_finger=boxes)
(root/'assets/open_gripper.json').write_text(json.dumps(out,indent=2)+'\n')
print('Generated',len(boxes),'boxes per finger')
