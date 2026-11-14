"""Read-only live exercise of the production final check against a saved ledger."""
import sys,json,argparse,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np,cv2,rospy,yaml
from scene1.rgbd import RgbdScene
from scene2.task import Sorter
from scene2.perception import Detector

parser=argparse.ArgumentParser()
parser.add_argument('--summary',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
          for p in [ROOT/'run_scene2.py',*sorted((ROOT/'scene2').glob('*.py'))]}
(args.output/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
rospy.init_node('scene2_verify_ledger',anonymous=True)
sorter=Sorter.__new__(Sorter)
sorter.scene=RgbdScene(rospy,yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment'])
sorter.detector=Detector()
ledger=json.loads(args.summary.read_text())
sorter.initial_counts=ledger['initial_bin_counts'];sorter.completed=ledger['completed'];rows=[]
def log(event,**values):
    rows.append(dict(event=event,**values));print(json.dumps(rows[-1]),flush=True)
def save(label,snapshot,parts,bins):
    prefix=args.output/f'{len(rows)+1:02d}'
    np.savez_compressed(str(prefix)+'.npz',**{k:snapshot[k] for k in
        ('rgb','depth_m','intrinsic','camera_to_base','stamp','table_z_base_m')})
    cv2.imwrite(str(prefix)+'.png',cv2.cvtColor(Detector.annotate(snapshot,parts,bins),cv2.COLOR_RGB2BGR))
sorter.log=log;sorter.save=save;result=dict(verified=False)
try:
    result['counts']=sorter.confirm_final();result['verified']=True
except Exception as error:
    result['error']=str(error)
finally:
    result['frames']=rows
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    sorter.scene.close()
raise SystemExit(0 if result['verified'] else 1)
