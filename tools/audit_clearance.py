import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import rospy,numpy as np
from scene1.kinematics import UrdfArmKinematics
from scene2.control import Robot,Stop
from scene2.perception import Part,Bin
rospy.init_node('scene2_clearance_audit',anonymous=True)
rows=[json.loads(x) for x in Path(sys.argv[1]).read_text().splitlines()]
obs=[r for r in rows if r['event']=='observation'][-1]
command=[r for r in rows if r['event']=='command'][-1]
r=Robot.__new__(Robot)
r.kin=UrdfArmKinematics(rospy);r.table=-.084
r.bins={k:Bin(**v) for k,v in obs['bins'].items()};r.parts=[Part(**p) for p in obs['parts']]
r.command=np.array(command['command']);r.side=1;r.target=None;r.held=False;r.stage='audit'
r.log=lambda event,**v:print(event,json.dumps(v,default=lambda x:x.tolist()))
for key in ('command','measured'):
    for side,start in enumerate((0,8)):
        mat=r.kin.matrix(np.array(command[key])[start:start+7],side)
        print(key,side,'tcp',mat[:3,3].tolist())
        try:r.check_clearance(mat,side)
        except Stop as e: print(e)
