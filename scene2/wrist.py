"""Registered wrist RGB-D with measured arm FK and public camera extrinsics."""
import time
import threading
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from scene1.rgbd import decode_depth


class WristScene:
    def __init__(self,rospy,head,kin):
        from sensor_msgs.msg import CameraInfo,CompressedImage
        self.rospy,self.head,self.kin=rospy,head,kin
        self.messages={}
        self.lock=threading.Lock()
        self.subs=[]
        self.calibration=[]
        for side,letter in enumerate(('l','r')):
            info=rospy.wait_for_message(f'/cam_{letter}/color/camera_info',CameraInfo,timeout=5)
            tr=head.buffer.lookup_transform(f'zarm_{letter}7_link',info.header.frame_id,rospy.Time(0),rospy.Duration(2))
            t,q=tr.transform.translation,tr.transform.rotation
            self.calibration.append((np.array([t.x,t.y,t.z]),Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),np.asarray(info.K).reshape(3,3)))
            for key,topic in [('rgb',f'/cam_{letter}/color/image_raw/compressed'),('depth',f'/cam_{letter}/depth/image_rect_raw/compressedDepth')]:
                self.subs.append(rospy.Subscriber(topic,CompressedImage,lambda m,s=side,k=key:self.receive(s,k,m),queue_size=1,buff_size=2**22))

    def receive(self,side,key,message):
        with self.lock: self.messages[side,key]=(message,time.monotonic())

    def read(self,side,table):
        with self.lock: messages=dict(self.messages)
        if any((side,k) not in messages for k in ('rgb','depth')): raise ValueError('Waiting for wrist RGB-D')
        if any(time.monotonic()-messages[side,k][1]>.8 for k in ('rgb','depth')): raise ValueError('Stale wrist RGB-D')
        rgb,depth=(messages[side,k][0] for k in ('rgb','depth'))
        stamp=rgb.header.stamp.to_sec()
        if abs(stamp-depth.header.stamp.to_sec())>.1: raise ValueError('Wrist RGB/depth mismatch')
        with self.head.lock: history=list(self.head.joint_history)
        if not history: raise ValueError('No arm history')
        joint=min(history,key=lambda m:abs(m.header.stamp.to_sec()-stamp))
        if abs(joint.header.stamp.to_sec()-stamp)>.035: raise ValueError('Wrist/arm timestamps mismatch')
        q=np.array(joint.joint_data.joint_q)[13+7*side:20+7*side]
        arm=self.kin.matrix(q,side)
        offset,rotation,k=self.calibration[side]
        matrix=np.eye(4)
        # Public simulator camera looks -Z and has upward image Y.
        matrix[:3,:3]=arm[:3,:3]@rotation@np.diag([1,-1,-1])
        matrix[:3,3]=arm[:3,3]-arm[:3,:3]@self.kin.tool_xyz+arm[:3,:3]@offset
        image=cv2.imdecode(np.frombuffer(rgb.data,np.uint8),cv2.IMREAD_COLOR)
        metres=decode_depth(depth)
        if image is None or metres.shape!=image.shape[:2]: raise ValueError('Wrist RGB-D dimensions mismatch')
        return dict(rgb=cv2.cvtColor(image,cv2.COLOR_BGR2RGB),depth_m=metres,intrinsic=k.copy(),camera_to_base=matrix,
                    stamp=stamp,table_z_base_m=float(table),source=f'wrist_{side}')

    def close(self):
        for sub in self.subs: sub.unregister()
