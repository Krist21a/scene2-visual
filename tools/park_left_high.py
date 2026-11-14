"""Empty left-hand recovery, lifting before a checked park path."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy,yaml,cv2
from dataclasses import replace
from scene1.rgbd import RgbdScene
from scene2.control import Robot
from scene2.task import Sorter
from scene2.perception import Bin
from scene2.perception import point_image,masks
rospy.init_node('park_left_high',anonymous=True)
def log(event,**kw):
    if event!='command':print(json.dumps(dict(event=event,**kw),default=lambda x:x.tolist()),flush=True)
s=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
r=sorter=None
try:
    closed_empty='--empty-closed' in sys.argv
    r=Robot(rospy,log,allow_loaded_recovery=closed_empty);sorter=Sorter(r,s,log,lambda *a:None)
    snapshot,_,bins=sorter.observe('recovery',{'A','B'});r.side=0
    if closed_empty:
        if r.measured()[7]<.97 or r.measured()[15]>.02:
            raise RuntimeError('Empty closed recovery requires fully closed left and open right jaws')
        tcp=r.pose(0)
        if tcp[2,3]<r.table+.13:raise RuntimeError('Empty closed recovery requires high hand')
        hp,hv=point_image(snapshot);hc,_=masks(snapshot['rgb'])
        on_table=hv&hc['C']&(hp[:,:,2]>r.table+.008)&(hp[:,:,2]<r.table+.04)&(hp[:,:,0]<bins['C'].lower[0]-.02)
        if on_table.sum()<100:raise RuntimeError('Dropped red handle is not visible on source table')
        for frame in (snapshot,sorter.wrist.read(0,r.table)):
            points,valid=point_image(frame);colours,_=masks(frame['rgb'])
            # RGB compression bleeds red across a finger occluding the distant
            # handle; boundary depth belongs to the finger, not red material.
            interior=cv2.erode(colours['C'].astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
            elevated=valid&interior&(points[:,:,2]>r.table+.055)&(np.linalg.norm(points-tcp[:3,3],axis=-1)<.08)
            log('empty_closed_view',source=frame.get('source','head'),tcp=tcp[:3,3],table=r.table,count=int(elevated.sum()),center=np.median(points[elevated],axis=0) if elevated.any() else None)
            if elevated.sum()>=12:raise RuntimeError('Elevated C material still near closed hand')
        r.check_clearance(tcp,0,extra_margin=.005,gripper_open=True)
        log('empty_closed_verified',gripper=r.measured()[7],tcp=tcp[:3,3])
        r.gripper(0,False,'empty_closed_reopen')
    if 'C' not in bins:
        rows=[json.loads(line) for line in Path(sys.argv[1]).read_text().splitlines()]
        previous=[e for e in rows if e['event']=='observation' and 'C' in e['bins']][-1]['bins']
        shifts=np.array([np.array(bins[k].center)-previous[k]['center'] for k in ('A','B')])
        shift=shifts.mean(0)
        if np.linalg.norm(shifts[0]-shifts[1])>.01 or np.linalg.norm(shift)>.03:
            raise RuntimeError('Recovery bin registration inconsistent')
        old=Bin(**previous['C']);dz=bins['A'].floor-previous['A']['floor']
        bins['C']=replace(old,center=(np.array(old.center)+shift).tolist(),
                          lower=(np.array(old.lower)+shift).tolist(),upper=(np.array(old.upper)+shift).tolist(),
                          floor=old.floor+dz,rim=old.rim+dz)
        log('registered_occluded_bin',shift=shift,dz=dz)
    pose=r.pose(0);point=pose[:3,3]+[0,0,.08]
    r.solve_pose(0,point,pose[:3,:3])
    r.cartesian(0,point,pose[:3,:3],'empty_lift')
    park=np.deg2rad([30,10,10,-120,-60,0,0]);initial=r.measured()[:7]
    for f in np.linspace(0,1,101):r.check_clearance(r.kin.matrix(initial+(park-initial)*f,0),0,extra_margin=.005)
    q=r.measured()[np.r_[0:7,8:15]];q[:7]=park
    r.joint_path(q,'empty_park')
finally:
    if sorter:sorter.close()
    s.close()
    if r:r.close()
