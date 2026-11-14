import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy
from scene1.kinematics import UrdfArmKinematics
from scene2.payload import screwdriver_payload
rospy.init_node('held_pose_replay',anonymous=True)
folder=Path(sys.argv[1]);rows=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
held=[r for r in rows if r['event']=='held_check' and r['stable']==3][-1]
q=[r for r in rows if r['event']=='command' and r['elapsed_s']<held['elapsed_s']][-1]['measured'][:7]
tcp=UrdfArmKinematics(rospy).matrix(q,0)
s=dict(np.load(sorted(folder.glob('*-held.npz'))[-1]))
payload,info=screwdriver_payload(s,tcp)
print(json.dumps(dict(payload=payload,info=info,tcp=tcp),default=lambda x:x.tolist(),indent=2))
