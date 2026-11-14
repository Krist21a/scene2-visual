"""Capture registered RGB-D and public calibration; never subscribe to truth."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import time
import cv2
import numpy as np
import yaml
import rospy
from scene1.rgbd import RgbdScene

rospy.init_node('scene2_observe', anonymous=True)
cfg = yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment']
scene = RgbdScene(rospy, cfg)
destination = Path(sys.argv[1])
destination.mkdir(parents=True, exist_ok=True)
deadline = time.monotonic()+25
try:
    while time.monotonic() < deadline:
        try:
            s = scene.read()
            np.savez_compressed(destination/'head.npz', **{k:s[k] for k in (
                'rgb','depth_m','intrinsic','camera_to_base','stamp','table_z_base_m')})
            cv2.imwrite(str(destination/'head.png'), cv2.cvtColor(s['rgb'], cv2.COLOR_RGB2BGR))
            print('Captured', destination, 'table', s['table_z_base_m'], flush=True)
            break
        except Exception as e:
            last = e
            time.sleep(.2)
    else:
        raise RuntimeError(f'No valid RGB-D: {last}')
finally:
    scene.close()
