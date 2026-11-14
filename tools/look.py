import sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bootstrap import ROOT
import numpy as np
import rospy
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot
rospy.init_node('scene2_look', anonymous=True)
assert KuavoSDK().Init()
robot = KuavoRobot()
for _ in range(20):
    robot.control_head(0., np.deg2rad(20))
    time.sleep(.1)
