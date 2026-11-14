"""Guarded development recovery after a stopped, still-open descent."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy,yaml
from dataclasses import replace
from scene1.rgbd import RgbdScene
from scene2.control import Robot,Stop
from scene2.task import Sorter,fits_bin
from scene2.perception import Part

checkpoint=Path(sys.argv[1]);output=Path(sys.argv[2]);output.mkdir(parents=True,exist_ok=False)
rows=[json.loads(l) for l in checkpoint.read_text().splitlines()]
command=next(e for e in reversed(rows) if e['event']=='command')
if command['stage']!='descend':raise Stop('Recovery requires interrupted open descent')
prior=next(e for e in reversed(rows) if e['event']=='observation')
part=Part(**next(e['part'] for e in reversed(rows) if e['event']=='target_refined'))
if part.kind!='A':raise Stop('This recovery is validated only for A')
stream=(output/'events.jsonl').open('w',buffering=1)
def log(event,**kw):
    text=json.dumps(dict(event=event,**kw),default=lambda x:x.tolist())
    stream.write(text+'\n')
    if event!='command':print(text,flush=True)
rospy.init_node('scene2_recover_empty_descent',anonymous=True)
scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
robot=sorter=None
try:
    robot=Robot(rospy,log);sorter=Sorter(robot,scene,log,lambda *a:None)
    snapshot,_,bins=sorter.observe('recovery')
    if np.any(robot.measured()[[7,15]]>.02):raise Stop('Recovery requires two open empty jaws')
    shifts=np.array([np.array(bins[k].center)-prior['bins'][k]['center'] for k in 'ABC'])
    shift=np.median(shifts,axis=0)
    if np.max(np.linalg.norm(shifts-shift,axis=1))>.015 or np.linalg.norm(shift)>.03:
        raise Stop('Recovery scene registration changed')
    dz=bins['A'].floor-prior['bins']['A']['floor']
    shift3=np.r_[shift,dz]
    part=replace(part,center=(np.array(part.center)+shift3).tolist(),
        full_center=(np.array(part.full_center or part.center)+shift3).tolist(),top=part.top+dz,
        footprint=(np.array(part.footprint)+shift).tolist(),
        occupied_xy=(np.array(part.occupied_xy)+shift).tolist())
    robot.parts=[p for p in robot.parts if not (p.kind=='A' and np.linalg.norm(np.array(p.center)-part.center)<.03)]+[part]
    all_parts,verification_bins=sorter.detector.detect(snapshot,True)
    robot.settled_parts=[p for p in all_parts if fits_bin(p,verification_bins[p.kind])]
    robot.side=1;robot.target=part
    pose=robot.pose(1);seed=robot.measured()[8:15].copy()
    points=[pose[:3,3]+[0,0,dz] for dz in np.linspace(0,.08,9)]
    for point in points:
        seed=robot.solve_pose(1,point,pose[:3,:3],seed)
        robot.check_clearance(robot.kin.matrix(seed,1),1)
    for point in points[1:]:robot.cartesian(1,point,pose[:3,:3],'descend',position_tolerance=.003)
    park=np.deg2rad([30,-10,-10,-120,60,0,0]);start=robot.measured()[8:15]
    for f in np.linspace(0,1,101):robot.check_clearance(robot.kin.matrix(start+(park-start)*f,1),1,extra_margin=.005)
    q=robot.measured()[np.r_[0:7,8:15]];q[7:]=park
    robot.joint_path(q,'recover_empty_park')
    log('recovered',checkpoint=str(checkpoint))
except Exception as e:
    log('stopped',error=str(e));raise
finally:
    if sorter:sorter.close()
    scene.close()
    if robot:robot.close()
    stream.close()
