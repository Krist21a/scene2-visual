import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import rospy,yaml,cv2,numpy as np
from scene1.rgbd import RgbdScene
from scene1.kinematics import UrdfArmKinematics
from scene2.wrist import WristScene
rospy.init_node('scene2_wrist_observe',anonymous=True)
head=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
time.sleep(2)
table=head.read()['table_z_base_m']
wrist=WristScene(rospy,head,UrdfArmKinematics(rospy))
time.sleep(2)
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
try:
    for side in (0,1):
        s=wrist.read(side,table)
        np.savez_compressed(out/f'wrist{side}.npz',**s)
        cv2.imwrite(str(out/f'wrist{side}.png'),cv2.cvtColor(s['rgb'],cv2.COLOR_RGB2BGR))
finally:
    wrist.close();head.close()
