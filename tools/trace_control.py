"""Read-only trace of the public MPC arm command path; run with ROS workspace sourced."""
import json
import sys
import threading
import time
import rospy
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import sensorsData,armTargetPoses
from ocs2_msgs.msg import mpc_target_trajectories

rospy.init_node('scene2_control_trace',anonymous=True)
lock=threading.Lock()
last={}
def record(kind,values):
    with lock:
        now=time.monotonic()
        if now-last.get(kind,0)<.2:return
        last[kind]=now
        print(json.dumps(dict(kind=kind,wall_time=time.time(),**values)),flush=True)

rospy.Subscriber('/kuavo_arm_traj',JointState,
    lambda m:record('sdk',dict(stamp=m.header.stamp.to_sec(),degrees=list(m.position))),queue_size=1)
rospy.Subscriber('/kuavo_arm_target_poses',armTargetPoses,
    lambda m:record('sdk_trajectory',dict(times=list(m.times),degrees=list(m.values))),queue_size=1)
rospy.Subscriber('/sensors_data_raw',sensorsData,
    lambda m:record('feedback',dict(stamp=m.header.stamp.to_sec(),q=list(m.joint_data.joint_q[13:27]))),queue_size=1)
rospy.Subscriber('/humanoid_mpc_arm_commanded',mpc_target_trajectories,
    lambda m:record('mpc',dict(times=list(m.timeTrajectory),q=[list(s.value) for s in m.stateTrajectory])),queue_size=1)
deadline=time.monotonic()+(float(sys.argv[1]) if len(sys.argv)>1 else 600.)
while not rospy.is_shutdown() and time.monotonic()<deadline:time.sleep(.2)
