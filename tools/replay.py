import sys, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import cv2
import numpy as np
from scene2.perception import Detector, serialise
s = dict(np.load(sys.argv[1]))
d = Detector()
parts,bins = d.detect(s)
print(json.dumps(serialise(parts,bins),indent=2))
cv2.imwrite(str(Path(sys.argv[1]).with_name('detected.png')),
            cv2.cvtColor(d.annotate(s,parts,bins),cv2.COLOR_RGB2BGR))
