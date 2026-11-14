"""Read-only IK and clearance survey from saved RGB-D and live joints."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy
from scipy.spatial.transform import Rotation
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.perception import Detector
rospy.init_node('scene2_grasp_survey',anonymous=True)
r=Robot.__new__(Robot);r.kin=UrdfArmKinematics(rospy)
msg=rospy.wait_for_message('/sensors_data_raw',sensorsData,timeout=5)
q=np.array(msg.joint_data.joint_q[13:27]);state=np.r_[q[:7],0,q[7:],0]
r.measured=lambda:state;r.command=state;r.held=False;r.payload=None;r.stage='survey';r.log=lambda *a,**k:None
s=dict(np.load(sys.argv[1]));r.table=float(s['table_z_base_m']);r.parts,r.bins=Detector().detect(s)
for p in r.parts:
 if len(sys.argv)>2 and p.kind!=sys.argv[2]:continue
 r.target=p
 a=np.array(p.axis);nominal=np.array([[a[1],a[0],0],[-a[0],a[1],0],[0,0,1]])
 for side in (0,1):
  r.side=side
  for yaw in np.arange(-np.pi,np.pi,np.pi/6):
   for tilt in (0,-.26,.26,-.52,.52):
    rot=nominal@Rotation.from_euler('z',yaw).as_matrix()@Rotation.from_euler('x',tilt).as_matrix()
    stage='grasp_ik'
    try:
     qg=r.solve_pose(side,np.array(p.center),rot)
     stage='clearance';r.check_clearance(r.kin.matrix(qg,side),side)
     stage='hover_ik';hover=np.array(p.center);hover[2]=r.table+.27;hover[0]+=.08;qa=r.solve_pose(side,hover,rot,qg)
     print(json.dumps(dict(kind=p.kind,center=p.center,side=side,yaw=yaw,tilt=tilt,status='ok',q=qg.tolist())),flush=True)
    except Stop as e:
     pass
