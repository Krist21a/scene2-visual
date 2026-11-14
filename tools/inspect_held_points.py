import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy
from scene1.kinematics import UrdfArmKinematics
from scene2.perception import point_image,masks
rospy.init_node('held_points_replay',anonymous=True)
f=Path(sys.argv[1]);rows=[json.loads(l) for l in (f/'events.jsonl').read_text().splitlines()];t=[r for r in rows if r['event']=='held_check' and r['stable']==3][-1]['elapsed_s'];q=[r for r in rows if r['event']=='command' and r['elapsed_s']<t][-1]['measured'][:7];tcp=UrdfArmKinematics(rospy).matrix(q,0);s=dict(np.load(sorted(f.glob('*-held.npz'))[-1]));p,v=point_image(s);c,_=masks(s['rgb']);table=float(s['table_z_base_m']);d=np.linalg.norm(p-tcp[:3,3],axis=-1)
for kind in ['C','A']:
 x=p[v&c[kind]&(d<.26)&(p[:,:,2]>table+.008)]
 center=np.median(x,axis=0);_,basis=np.linalg.eigh(np.cov(x.T));ext=np.diff(np.percentile((x-center)@basis,[1,99],axis=0),axis=0)[0]
 print(kind,len(x),center,ext,basis[:,-1]);np.save('/tmp/held-'+kind+'.npy',x)
print('tcp',tcp)
