"""Read-only route survey using a saved RGB-D observation and public kinematics."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy
from scipy.spatial.transform import Rotation
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.task import Sorter
from scene2.perception import Detector
rospy.init_node('scene2_place_survey',anonymous=True)
r=Robot.__new__(Robot);r.kin=UrdfArmKinematics(rospy)
msg=rospy.wait_for_message('/sensors_data_raw',sensorsData,timeout=5)
q=np.array(msg.joint_data.joint_q[13:27]);state=np.r_[q[:7],0,q[7:],0]
r.measured=lambda:state;r.command=state;r.held=False;r.payload=None;r.stage='survey'
r.log=lambda event,**v:None
s=dict(np.load(sys.argv[1]));r.table=float(s['table_z_base_m']);r.parts,r.bins=Detector().detect(s)
p=sorted([p for p in r.parts if p.kind==sys.argv[2]],key=lambda p:p.center[1])[0]
r.target=p;r.side=1 if p.center[1]<0 else 0
sorter=Sorter.__new__(Sorter);sorter.robot=r;sorter.completed=[];sorter.initial_counts=dict.fromkeys('ABC',0)
sorter.log=lambda event,**v:print(event,json.dumps(v,default=lambda x:x.tolist()),flush=True)
a=np.array(p.axis);nominal=np.array([[a[1],a[0],0],[-a[0],a[1],0],[0,0,1]])
rot=nominal@Rotation.from_euler('z',np.pi).as_matrix()@Rotation.from_euler('x',.52).as_matrix()
try:
 route=sorter.plan_place(p,r.bins[p.kind],rot,np.array(p.center),max(b.rim for b in r.bins.values())+.11)
 print('ROUTE_FOUND',flush=True)
except Stop as e:print('REJECTED',str(e),flush=True)
