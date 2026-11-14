import sys,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy,yaml
from scipy.spatial.transform import Rotation
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData
from scene1.kinematics import UrdfArmKinematics
from scene1.rgbd import RgbdScene
from scene2.wrist import WristScene
from scene2.control import Robot,Stop
from scene2.perception import Detector
rospy.init_node('camera_view_survey',anonymous=True)
head=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment']);kin=UrdfArmKinematics(rospy);wrist=WristScene(rospy,head,kin);time.sleep(2)
msg=rospy.wait_for_message('/sensors_data_raw',sensorsData,timeout=5);q=np.array(msg.joint_data.joint_q[13:27]);state=np.r_[q[:7],0,q[7:],0]
s=head.read();r=Robot.__new__(Robot);r.kin=kin;r.measured=lambda:state;r.command=state;r.log=lambda *a,**kw:None;r.table=s['table_z_base_m'];r.parts,r.bins=Detector().detect(s);r.target=None;r.side=0;r.held=False;r.payload=None;r.stage='survey'
rows=[json.loads(l) for l in Path(sys.argv[1]).read_text().splitlines()];payload=[x for x in rows if x['event']=='payload_measured'][-1];left=kin.matrix(q[:7],0);target=left[:3,3]+left[:3,:3]@payload['local_center'];offset,relative,_=wrist.calibration[1];relative=relative@np.diag([1,-1,-1]);options=[]
for x in (.3,.4,.5):
 for y in (-.1,-.2):
  for z in (.10,.20,.30):
   camera=np.array([x,y,z]);forward=target-camera;forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right);base=np.column_stack((right,np.cross(forward,right),forward))
   for roll in (0.,np.pi/4,-np.pi/4,np.pi/2,-np.pi/2,np.pi):
    rot=base@Rotation.from_euler('z',roll).as_matrix()@relative.T;pos=camera-rot@(offset-kin.tool_xyz)
    if pos[1]>0:continue
    try:
     solution=r.solve_pose(1,pos,rot,q[7:]);r.check_clearance(kin.matrix(solution,1),1)
     for t in np.linspace(0,1,31):r.check_clearance(kin.matrix(q[7:]*(1-t)+solution*t,1),1)
     options.append(dict(q=solution.tolist(),camera=camera.tolist(),tool=pos.tolist(),score=float(np.linalg.norm(solution-q[7:]))));print('FOUND',options[-1],flush=True)
    except Stop:pass
print(json.dumps(dict(target=target.tolist(),options=sorted(options,key=lambda v:v['score'])),indent=2))
wrist.close();head.close()
