"""Recover an open left hand over checked joint paths from recorded waypoints."""
import sys,json,heapq
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,rospy,yaml
from scene1.rgbd import RgbdScene
from scene2.control import Robot,Stop
from scene2.task import Sorter

rospy.init_node('park_left_checked',anonymous=True)
def log(event,**kw):
    if event!='command':print(json.dumps(dict(event=event,**kw),default=lambda x:x.tolist()),flush=True)
scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
r=sorter=None
try:
    rows=[json.loads(l) for l in Path(sys.argv[1]).read_text().splitlines()]
    r=Robot(rospy,log);sorter=Sorter(r,scene,log,lambda *a:None)
    sorter.observe('empty_recovery');r.side=0
    nodes=[r.measured()[:7],np.deg2rad([30,10,10,-120,-60,0,0])]
    # Only the last C operation supplies waypoints; earlier A/B used other arms.
    first=max(i for i,e in enumerate(rows) if e.get('event')=='select')
    for e in rows[first:]:
        if e.get('event')=='place_plan':nodes.extend([np.array(e['high_q']),np.array(e['q'])])
    queue=[(0.,0,[])];visited=set();route=None
    while queue:
        cost,i,path=heapq.heappop(queue)
        if i in visited:continue
        visited.add(i)
        if i==1:route=path;break
        for j,b in enumerate(nodes):
            if j in visited:continue
            a=nodes[i]
            try:
                for fraction in np.linspace(0,1,max(31,int(np.ceil(np.max(np.abs(b-a))/.02))+1)):
                    r.check_clearance(r.kin.matrix(a+(b-a)*fraction,0),0,extra_margin=.005,gripper_open=True)
            except Stop:continue
            heapq.heappush(queue,(cost+float(np.linalg.norm(b-a)),j,path+[j]))
    if route is None:raise Stop('No checked empty-hand route to park')
    log('empty_recovery_route',waypoints=[nodes[i] for i in route])
    for i in route:
        q14=r.measured()[np.r_[0:7,8:15]];q14[:7]=nodes[i]
        r.joint_path(q14,'empty_recovery_park')
    log('empty_recovery_complete')
finally:
    if sorter:sorter.close()
    scene.close()
    if r:r.close()
