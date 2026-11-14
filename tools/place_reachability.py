import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy
from scipy.spatial.transform import Rotation
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
rospy.init_node('scene2_place_reachability',anonymous=True)
r=Robot.__new__(Robot);r.kin=UrdfArmKinematics(rospy)
msg=rospy.wait_for_message('/sensors_data_raw',sensorsData,timeout=5)
q14=np.array(msg.joint_data.joint_q[13:27]);q=np.r_[q14[:7],0,q14[7:],1]
r.measured=lambda:q
rows=[json.loads(x) for x in Path(sys.argv[1]).read_text().splitlines()]
obs=[x for x in rows if x['event']=='observation'][-1]
b=obs['bins']['B'];xyz=np.r_[b['center'],b['rim']+.07];xyz[0]-=.05
for tilt in (-.52,-.78,-1.05,-1.3):
    for yaw in (0,np.pi/2,np.pi,-np.pi/2):
        rot=Rotation.from_euler('y',tilt).as_matrix()@Rotation.from_euler('z',yaw).as_matrix()
        try:
            solution=r.solve_pose(1,xyz,rot)
            print(json.dumps(dict(tilt=tilt,yaw=yaw,xyz=xyz.tolist(),q=solution.tolist())),flush=True)
        except Stop: pass
