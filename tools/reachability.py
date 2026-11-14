import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import rospy
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData
from scene1.kinematics import UrdfArmKinematics
from scene2.perception import Detector
rospy.init_node('scene2_reachability',anonymous=True)
kin=UrdfArmKinematics(rospy)
msg=rospy.wait_for_message('/sensors_data_raw',sensorsData,timeout=5)
q=np.array(msg.joint_data.joint_q[13:27])
s=dict(np.load(sys.argv[1]))
parts,bins=Detector().detect(s)
print('limits',np.rad2deg(np.vstack(kin.limits)).tolist())
print('measured',q.tolist())
for p in parts:
    side=1 if p.center[1]<0 else 0
    start=q[side*7:side*7+7]
    a=np.array(p.axis)
    nominal=np.array([[a[1],a[0],0],[-a[0],a[1],0],[0,0,1]])
    candidates=[]
    for yaw in (0,np.pi,np.pi/2,-np.pi/2):
        rot=nominal@Rotation.from_euler('z',yaw).as_matrix()
        def residual(j):
            mat=kin.matrix(j,side)
            return np.r_[mat[:3,3]-p.center,.15*Rotation.from_matrix(rot@mat[:3,:3].T).as_rotvec()]
        best=None
        for seed in (start,np.mean(kin.limits[side],axis=1)):
            out=least_squares(residual,np.clip(seed,kin.limits[side][:,0]+.0011,kin.limits[side][:,1]-.0011),bounds=(kin.limits[side][:,0]+.001,kin.limits[side][:,1]-.001),max_nfev=250)
            err=residual(out.x)
            item=dict(yaw=yaw,pos=float(np.linalg.norm(err[:3])),ori=float(np.linalg.norm(err[3:])/.15),q=out.x.tolist())
            if best is None or np.linalg.norm(err)<best[0]: best=(np.linalg.norm(err),item)
        candidates.append(best[1])
    print(json.dumps(dict(kind=p.kind,center=p.center,options=candidates)))
