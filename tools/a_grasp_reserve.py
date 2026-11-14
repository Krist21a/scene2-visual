"""Read-only grasp reserve survey from recorded RGB-D and public URDF."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy
from scipy.spatial.transform import Rotation as R
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin

folder=Path(sys.argv[1])
rows=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
c=next(e for e in reversed(rows) if e['event']=='command')
o=next(e for e in reversed(rows) if e['event']=='observation')
p=Part(**next(e['part'] for e in reversed(rows) if e['event']=='target_refined'))
assert p.kind=='A'
r=Robot.__new__(Robot);r.kin=UrdfArmKinematics(rospy)
r.command=np.array(c['command']);r.measured=lambda:np.array(c['measured'])
r.table=float(np.load(sorted(folder.glob('*-pregrasp.npz'))[-1])['table_z_base_m'])
r.bins={a:Bin(**v) for a,v in o['bins'].items()}
r.parts=[p];r.target=p;r.side=1;r.held=False;r.payload=None;r.stage='survey'
r.log=lambda *a,**kw:None
a=np.array(p.axis)
nominal=np.array([[a[1],a[0],0],[-a[0],a[1],0],[0,0,1]])
for height in (.004,.008,.012):
    successes=[];clear=0
    for along,across in [(0,0),(-p.extent[0]*.35,0),(p.extent[0]*.35,0),(0,.012),(0,-.012),(.015,0),(-.015,0)]:
        xyz=np.array(p.center)
        xyz[:2]+=a*along+np.array([-a[1],a[0]])*across
        xyz[2]=max(p.center[2]+.004,r.table+.013)+height
        for yaw in np.arange(0,2*np.pi,np.pi/6):
            for tilt in (0.,-.26,.26,-.52,.52,-.78,.78):
                rot=nominal@R.from_euler('z',yaw).as_matrix()@R.from_euler('x',tilt).as_matrix()
                mat=np.eye(4);mat[:3,:3]=rot;mat[:3,3]=xyz
                try:r.check_clearance(mat,1,extra_margin=.002)
                except Stop:continue
                clear+=1
                try:
                    q=r.solve_pose(1,xyz,rot)
                    r.check_clearance(r.kin.matrix(q,1),1,extra_margin=.002)
                    hover=xyz.copy();hover[2]=r.table+.12
                    r.solve_pose(1,hover,rot,q);r.transfer_pose(1,p,xyz,rot,q)
                    successes.append(dict(point=xyz.tolist(),yaw=float(yaw),tilt=tilt,along=along,across=across))
                except Stop:continue
    print(json.dumps(dict(height=height,clear=clear,reachable=successes)),flush=True)
