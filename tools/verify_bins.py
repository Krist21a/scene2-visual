"""Independent read-only multi-frame bin occupancy check from head RGB-D."""
import sys,json,time,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,cv2,yaml,rospy
from scene1.rgbd import RgbdScene
from scene2.perception import Detector,serialise
from scene2.task import fits_bin
ap=argparse.ArgumentParser();ap.add_argument('--expected',required=True);ap.add_argument('--output',required=True)
ap.add_argument('--after-stamp',type=float,help='Require frames newer than the last production check');a=ap.parse_args()
expected={k:int(v) for k,v in (x.split(':') for x in a.expected.split(','))};dest=Path(a.output);dest.mkdir(parents=True,exist_ok=False)
rospy.init_node('scene2_verify_bins',anonymous=True)
scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
rows=[];last=a.after_stamp;deadline=time.monotonic()+30
try:
 while len(rows)<5 and time.monotonic()<deadline:
  try:s=scene.read()
  except (ValueError,RuntimeError):time.sleep(.2);continue
  if last is not None and s['stamp']<=last:time.sleep(.2);continue
  last=s['stamp'];parts,bins=Detector().detect(s,True)
  counts={k:sum(p.kind==k and fits_bin(p,bins[k]) for p in parts) if k in bins else None for k in 'ABC'}
  row=dict(stamp=s['stamp'],counts=counts,**serialise(parts,bins));rows.append(row);print(json.dumps(row),flush=True)
  np.savez_compressed(dest/f'{len(rows):02d}.npz',**{k:s[k] for k in ('rgb','depth_m','intrinsic','camera_to_base','stamp','table_z_base_m')})
  cv2.imwrite(str(dest/f'{len(rows):02d}.png'),cv2.cvtColor(Detector.annotate(s,parts,bins),cv2.COLOR_RGB2BGR));time.sleep(.3)
finally:scene.close()
ok=len(rows)==5 and all(all(r['counts'][k]==v for k,v in expected.items()) for r in rows)
(dest/'summary.json').write_text(json.dumps(dict(verified=ok,expected=expected,frames=rows),indent=2)+'\n')
raise SystemExit(0 if ok else 1)
