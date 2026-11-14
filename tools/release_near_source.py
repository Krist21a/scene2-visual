"""Development recovery: open only when still immediately above observed source."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy,yaml
from dataclasses import replace
from scene1.rgbd import RgbdScene
from scene2.control import Robot,Stop
from scene2.task import Sorter
from scene2.perception import Part
rospy.init_node('release_near_source',anonymous=True)
def log(event,**kw):
    if event!='command':print(json.dumps(dict(event=event,**kw),default=lambda x:x.tolist()),flush=True)
scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
r=sorter=None
try:
    rows=[json.loads(l) for l in Path(sys.argv[1]).read_text().splitlines()]
    part=Part(**[e['part'] for e in rows if e['event']=='target_refined'][-1])
    old=[e for e in rows if e['event']=='observation'][-1]['bins']
    r=Robot(rospy,log,allow_loaded_recovery=True);sorter=Sorter(r,scene,log,lambda *a:None)
    _,_,bins=sorter.observe('source_release_check')
    shifts=np.array([np.array(bins[k].center)-old[k]['center'] for k in ('A','B')])
    shift=shifts.mean(0)
    if np.linalg.norm(shifts[0]-shifts[1])>.01 or np.linalg.norm(shift)>.03:raise Stop('Source registration inconsistent')
    source=np.array(part.center);source[:2]+=shift
    pose=r.pose(0)
    if np.linalg.norm(pose[:2,3]-source[:2])>.03 or not .015<pose[2,3]-r.table<.08:
        raise Stop('Loaded hand is not immediately above observed source')
    if r.measured()[15]>.02:raise Stop('Opposite gripper is not empty')
    r.side=0;r.target=part;r.held=True
    log('source_release_authorised_geometry',tcp=pose[:3,3],source=source)
    r.gripper(0,False,'source_release');r.held=False;r.target=None
    pose=r.pose(0);point=pose[:3,3]+[0,0,.08]
    r.solve_pose(0,point,pose[:3,:3]);r.cartesian(0,point,pose[:3,:3],'empty_source_retract')
finally:
    if sorter:sorter.close()
    scene.close()
    if r:r.close()
