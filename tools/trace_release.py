"""Read-only RGB-D capture triggered by a policy's verified release event."""
import sys,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import cv2,numpy as np,yaml,rospy
from scene1.rgbd import RgbdScene
from scene2.control import UrdfArmKinematics
from scene2.wrist import WristScene

rospy.init_node('release_sensor_trace',anonymous=True)
scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
wrist=WristScene(rospy,scene,UrdfArmKinematics(rospy))
run=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=False)
deadline=time.monotonic()+900
try:
    with (run/'events.jsonl').open() as events:
        triggered=False
        while time.monotonic()<deadline:
            line=events.readline()
            if not line:
                time.sleep(.05);continue
            e=json.loads(line)
            if e.get('event')=='release_payload_verified' and e.get('stable')==3:
                triggered=True;break
        if not triggered:raise RuntimeError('No release trigger')
    until=time.monotonic()+45;last=None;index=0
    while time.monotonic()<until:
        try:
            s=scene.read(candidate_stride=4)
            if s['stamp']==last:continue
            last=s['stamp'];index+=1
            np.savez_compressed(out/f'{index:03}.npz',**{k:s[k] for k in (
                'rgb','depth_m','intrinsic','camera_to_base','stamp','table_z_base_m')})
            cv2.imwrite(str(out/f'{index:03}.png'),cv2.cvtColor(s['rgb'],cv2.COLOR_RGB2BGR))
            try:
                w=wrist.read(1,s['table_z_base_m'])
                np.savez_compressed(out/f'{index:03}-wrist.npz',**w)
                cv2.imwrite(str(out/f'{index:03}-wrist.png'),cv2.cvtColor(w['rgb'],cv2.COLOR_RGB2BGR))
            except ValueError:pass
        except (ValueError,RuntimeError):time.sleep(.05)
    print('Captured',index,'release frames',flush=True)
finally:
    wrist.close()
    scene.close()
