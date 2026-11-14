"""Bounded Cartesian feedback through the inherited MPC arm command channel."""
import time
from .timing import timed
import json
from pathlib import Path
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from scipy.optimize import lsq_linear, least_squares
from scene1.motion import MotionFilter, JOINTS
from scene1.kinematics import UrdfArmKinematics
from scene1.clearance import boxes_overlap


class Stop(RuntimeError):
    pass


A_LIFT_JOINT_MARGIN=np.array([.003,.003,.003,.003,.003,.08,.003])


def feedback_joint_bounds(limits,seed,joint_margin,reference_bias=None):
    lower,upper=limits[:,0]+joint_margin,limits[:,1]-joint_margin
    if reference_bias is not None:
        bias=np.asarray(reference_bias)
        if not np.isfinite(bias).all() or np.max(np.abs(bias))>.25:
            raise Stop('Planning reference bias exceeds tracking limit')
        # Keep room for the measured MPC compensation as well as the existing
        # geometric margin. A saturated actuator must not supply a new pose.
        lower=np.maximum(lower,limits[:,0]+.010-bias)
        upper=np.minimum(upper,limits[:,1]-.010-bias)
        saturated=np.abs(bias)>.215
        lower=np.where(saturated,np.maximum(lower,seed-.0001),lower)
        upper=np.where(saturated,np.minimum(upper,seed+.0001),upper)
    if np.any(lower>=upper):raise Stop('No joint range with measured compensation reserve')
    return lower,upper


def rotation_log_right_inverse(vector):
    """Derivative of Log(R Exp(delta)) with respect to delta, away from pi."""
    x,y,z=vector
    skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    angle=np.linalg.norm(vector)
    coefficient=(1/12+angle*angle/720 if angle<1e-4 else
                 (1-.5*angle/np.tan(.5*angle))/(angle*angle))
    return np.eye(3)+.5*skew+coefficient*(skew@skew)


@lru_cache(maxsize=1)
def open_finger_geometry():
    data=json.loads((Path(__file__).resolve().parents[1]/'assets/open_gripper.json').read_text())
    return [(np.asarray(row['center'])*[sign,1,1],np.asarray(row['half']))
            for sign in (-1,1) for row in data['positive_finger']]


def prism_overlap(center,rotation,half,footprint,bottom,top):
    """SAT for an observed convex footprint extruded vertically and a hand OBB."""
    xy=np.asarray(footprint)
    vertices=np.vstack((np.column_stack((xy,np.full(len(xy),bottom))),
                        np.column_stack((xy,np.full(len(xy),top)))))
    edges=np.column_stack((np.roll(xy,-1,axis=0)-xy,np.zeros(len(xy))))
    vertical=np.array([0.,0.,1.])
    axes=[vertical,*rotation.T]
    for edge in [vertical,*edges]:
        axes.extend(np.cross(edge,axis) for axis in rotation.T)
    axes.extend(np.cross(edge,vertical) for edge in edges)
    for axis in axes:
        length=np.linalg.norm(axis)
        if length<1e-8:continue
        axis=axis/length;projected=vertices@axis
        middle=np.dot(center,axis);radius=np.abs(rotation.T@axis)@half
        if middle+radius<projected.min() or middle-radius>projected.max():return False
    return True


def columns_overlap(center,rotation,half,xy,bottom,top):
    """Conservative 5mm columns covering the observed mesh silhouette."""
    columns=np.column_stack((xy,np.full(len(xy),(bottom+top)/2)))
    column_half=np.array([.0025,.0025,(top-bottom)/2])
    axes=[*np.eye(3),*rotation.T]
    axes.extend(np.cross(a,b) for a in np.eye(3) for b in rotation.T)
    axes=np.array([a/np.linalg.norm(a) for a in axes if np.linalg.norm(a)>1e-8])
    radius=np.abs(axes@rotation)@half+np.abs(axes)@column_half
    return bool(np.any(np.all(np.abs((columns-center)@axes.T)<=radius,axis=1)))


class Robot:
    def __init__(self, rospy, log, allow_loaded_recovery=False):
        from kuavo_deploy.config import load_kuavo_config
        from kuavo_deploy.kuavo_env.KuavoSimEnv import KuavoSimEnv
        from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import sensorsData
        from sensor_msgs.msg import JointState
        cfg = load_kuavo_config('/root/LeTools-Learning/configs/deploy/deploy_scene1.yaml')
        cfg.env.ros_rate = 10
        cfg.env.direct_to_wbc = False
        cfg.env.head_init = np.deg2rad([0,20]).tolist()
        if (cfg.env.inference_env,cfg.env.only_arm,cfg.env.which_arm,cfg.env.eef_type) != ('sim',True,'both','rq2f85'):
            raise Stop('Unexpected robot configuration')
        self.rospy, self.log = rospy, log
        self.received = {}
        self.fault=None
        self.bad_tilt_since=None
        self.run_id=rospy.get_param('/run_id')
        self.description=rospy.get_param('/humanoid_description')
        self.subs = [rospy.Subscriber(topic, cls, lambda m,k=key:self._receive(k,m),queue_size=1)
                     for topic,cls,key in [('/sensors_data_raw',sensorsData,'joints'),
                                           ('/gripper/state',JointState,'grippers')]]
        self.env = KuavoSimEnv(cfg)
        from kuavo_deploy.kuavo_env.KuavoBaseRosEnv import log_robot
        import logging
        log_robot.setLevel(logging.WARNING)
        self.env._set_direct_to_wbc(0)
        self.env.reset()
        self.kin = UrdfArmKinematics(rospy)
        sdk_low,sdk_high=self.env.robot._robot_info.get_arm_joint_limits()
        physical=np.vstack(self.kin.limits)
        if not np.allclose(np.column_stack((sdk_low,sdk_high)),physical,atol=1e-8):
            raise Stop('SDK/URDF limits disagree')
        self.low,self.high=self.env.action_space.low.copy(),self.env.action_space.high.copy()
        self.low[JOINTS]=np.maximum(self.low[JOINTS],physical[:,0])
        self.high[JOINTS]=np.minimum(self.high[JOINTS],physical[:,1])
        self.command = self.measured()
        if np.any(self.command[[7,15]]>.02) and not allow_loaded_recovery:
            raise Stop('Initial grippers are closed or loaded; use a complete scene reset')
        if not allow_loaded_recovery:self.command[[7,15]] = 0.
        self.filter = MotionFilter(self.low,self.high,1.,.12)
        self.filter.reset(self.command)
        self.command[JOINTS]=self.filter.previous[JOINTS]
        self.table = None
        self.bins = {}
        self.parts = []
        self.settled_parts = []
        self.target = None
        self.side = None
        self.held = False
        self.retained=False
        self.contact_loss_steps=0
        self.payload = None
        self.stage = 'initialise'
        self.steps = 0

    def _receive(self, key, message):
        self.received[key] = time.monotonic()
        if key=='joints':
            q=message.imu_data.quat
            values=np.array([q.x,q.y,q.z,q.w])
            if not np.isfinite(values).all() or np.linalg.norm(values)<.5:
                self.fault='Invalid IMU feedback'
                return
            roll,pitch,_=Rotation.from_quat(values).as_euler('xyz')
            if max(abs(roll),abs(pitch))>np.deg2rad(20):
                if self.bad_tilt_since is None: self.bad_tilt_since=time.monotonic()
                elif time.monotonic()-self.bad_tilt_since>.1: self.fault='Base tilt exceeded 20 degrees'
            else: self.bad_tilt_since=None

    def measured(self):
        if self.rospy.is_shutdown():
            raise Stop('ROS shutdown')
        if self.fault: raise Stop(self.fault)
        if any(time.monotonic()-self.received.get(k,0)>1. for k in ('joints','grippers')):
            raise Stop('Stale joint/gripper feedback')
        q = self.env.get_obs()['observation.state'].detach().cpu().numpy().reshape(-1).astype(float)
        if q.shape != (16,) or not np.isfinite(q).all():
            raise Stop('Invalid joint/gripper feedback')
        return q

    def emit(self, goal, guard=True, backtrack=False):
        if (self.rospy.get_param_cached('/run_id')!=self.run_id or
                self.rospy.get_param_cached('/humanoid_description')!=self.description):
            raise Stop('ROS epoch or robot geometry changed')
        measured = self.measured()
        if self.retained and self.side is not None:
            self.contact_loss_steps=(self.contact_loss_steps+1 if measured[(7,15)[self.side]]>=.97 else 0)
            if self.contact_loss_steps>=2:raise Stop('Lost gripper contact during transport')
        if guard and self.table is not None:
            for side,start in enumerate((0,8)):
                self.check_clearance(self.kin.matrix(measured[start:start+7],side),side)
        previous=self.filter.previous.copy()
        scales=(1.,.5,.25,.125,.0625,0.) if backtrack and guard else (1.,)
        rejection=None
        for scale in scales:
            # Rejected previews must not advance the slew limiter's state.
            self.filter.previous=previous.copy()
            trial=np.asarray(goal).copy() if scale==1. else self.command+scale*(np.asarray(goal)-self.command)
            cmd,info=self.filter.apply(trial,.1)
            if info['bounds_clipped']:
                self.filter.previous=previous
                raise Stop('Requested command outside limits')
            if np.max(np.abs(cmd[JOINTS]-measured[JOINTS]))>.25:
                self.filter.previous=previous
                raise Stop('Arm reference tracking error exceeds 0.25 rad')
            try:
                if guard and self.table is not None:
                    for side,start in enumerate((0,8)):
                        for alpha in (.5,1.):
                            mat=self.kin.matrix(measured[start:start+7]*(1-alpha)+cmd[start:start+7]*alpha,side)
                            self.check_clearance(mat,side)
                break
            except Stop as e:
                rejection=e
        else:
            self.filter.previous=previous
            raise rejection
        if scale<1.:
            self.log('reference_limited',stage=self.stage,scale=scale,reason=str(rejection))
        self.env.exec_action(cmd)
        self.command = cmd
        self.steps += 1
        self.log('command',stage=self.stage,step=self.steps,command=cmd,measured=measured)
        time.sleep(.1)

    def check_clearance(self, mat, side, payload=None, extra_margin=0., gripper_open=None, table_reserve=0.):
        # Conservative simplified fingertip/palm checks; not a whole-body proof.
        finger_half = np.array([.015852,.011,.028258])
        if gripper_open is None:gripper_open=self.command[(7,15)[side]]<.3
        geometries=(list(open_finger_geometry()) if gripper_open else
                    [(np.array([s*.025,0,.025857]),finger_half) for s in (-1,1)])
        geometries += [(np.array([0,0,.105]),np.array([.045,.035,.035]))]
        world=[(mat[:3,3]+mat[:3,:3]@offset,mat[:3,:3],half,False) for offset,half in geometries]
        carried=payload if payload is not None else (self.payload if self.held and side==self.side else None)
        if carried is not None:
            offset,rotation,half=carried
            if self.target is not None and self.target.kind=='C':
                length=2*half[0]
                handle=min(self.target.handle_length,length)
                # The observed red handle and thin metal shaft have different
                # widths. Full-length box remains the bin-containment envelope.
                # In-hand estimates orient +X toward the metal tip.
                sign=getattr(self,'payload_shaft_sign',1.)
                for shift,segment_half in (
                        (-sign*(length-handle)/2,np.array([handle/2,half[1],half[2]])),
                        (sign*handle/2,np.array([(length-handle)/2,.004,.004]))):
                    segment=offset+rotation@np.array([shift,0.,0.])
                    world.append((mat[:3,3]+mat[:3,:3]@segment,mat[:3,:3]@rotation,segment_half,True))
            else:
                world.append((mat[:3,3]+mat[:3,:3]@offset,mat[:3,:3]@rotation,half,True))
        for center,geometry_rotation,half,is_payload in world:
            half=half+extra_margin
            world_half=np.abs(geometry_rotation)@half+.003
            low_z = center[2]-np.abs(geometry_rotation[2,:3])@half
            margin=-.003 if is_payload and self.stage=='lift' else .003
            if low_z < self.table+margin+table_reserve:
                raise Stop(f'Hand/table clearance failed: {low_z-self.table:.4f} m')
            for b in self.bins.values():
                lo,hi = np.asarray(b.lower),np.asarray(b.upper)
                if (np.any(center+world_half<np.r_[lo-.027,self.table]) or
                        np.any(center-world_half>np.r_[hi+.027,b.rim])):
                    continue
                if center[2]-np.linalg.norm(half) >= b.rim+.003:
                    continue
                # Include wall strips only; floor polygon is a conservative inner area.
                for axis in (0,1):
                    for bound in (lo[axis]-.012,hi[axis]+.012):
                        wall_center = np.r_[(lo+hi)/2,(self.table+b.rim)/2]
                        wall_center[axis] = bound
                        wall_half = np.r_[(hi-lo)/2+.024,(b.rim-self.table)/2]
                        wall_half[axis] = .015
                        if boxes_overlap(center,geometry_rotation,half+.003,wall_center,np.eye(3),wall_half):
                            self.log('blocked_wall',stage=self.stage,side=side,bin=b.kind,payload=is_payload,
                                     tcp=mat[:3,3],geometry_center=center,wall_center=wall_center)
                            raise Stop('Hand/bin wall clearance failed')
            obstacles=[(p,self.table,False) for p in self.parts]
            obstacles += [(p,self.bins[p.kind].floor,True)
                          for p in getattr(self,'settled_parts',[]) if p.kind in self.bins]
            for p,bottom,settled in obstacles:
                if self.target is p and side == self.side and (self.held or self.stage=='close'):
                    continue
                # The intended target may occupy the gap, never the fingers.
                axis = np.array(p.axis)
                rot = np.array([[axis[0],-axis[1],0],[axis[1],axis[0],0],[0,0,1]])
                size = np.r_[p.full_extent or p.extent,(p.top-bottom)]/2
                object_center=np.array(p.full_center or p.center).copy()
                if settled:object_center[2]=(bottom+p.top)/2
                if p.footprint:
                    footprint=np.asarray(p.footprint)
                    object_lo=np.r_[footprint.min(0)-.003,bottom]
                    object_hi=np.r_[footprint.max(0)+.003,p.top]
                    if np.any(center+world_half<object_lo) or np.any(center-world_half>object_hi):continue
                overlap=(columns_overlap(center,geometry_rotation,half+.003,p.occupied_xy,bottom,p.top)
                         if self.target is p and p.kind=='A' and p.occupied_xy else
                         prism_overlap(center,geometry_rotation,half+.003,p.footprint,bottom,p.top)
                         if p.footprint else boxes_overlap(center,geometry_rotation,half+.003,object_center,rot,size))
                if overlap:
                    self.log('blocked_geometry',side=side,tcp=mat[:3,3],rotation=mat[:3,:3],finger=center,
                             finger_half=half,object_center=p.full_center or p.center,object_size=size,
                             object_axis=p.axis,intended=self.target is p)
                    raise Stop(f'Hand/part clearance failed ({p.kind})')

    def check_transport_start(self, state, payload):
        start=(0,8)[self.side]
        arm=state[start:start+7]
        self.check_clearance(self.kin.matrix(arm,self.side),self.side,payload=payload,extra_margin=.010)
        for alpha in (.5,1.):
            reference=arm+(self.command[start:start+7]-arm)*alpha
            self.check_clearance(self.kin.matrix(reference,self.side),self.side,payload=payload)

    def transport_clearance_goal(self, state, payload):
        """Find a small retreat whose first bounded command is already clear.

        A newly measured payload can invalidate the old compensated reference.
        Never bypass emit's checks: reject even a useful retreat when its first
        command cannot clear the new envelope within the existing slew limit.
        """
        side=self.side;start=(0,8)[side];arm=state[start:start+7]
        pose,jac=self.kin.matrix_and_jacobian(arm,side)
        bias=self.command[start:start+7]-arm
        for offset in ([0.,0.,.025],[-.025,0.,.025],[-.025,0.,0.],[-.025,0.,.05]):
            point=pose[:3,3]+offset
            try:
                dq=self._cartesian_delta(state,start,jac,np.r_[offset,[0.,0.,0.]],.20)
                reference=self.command[start:start+7]+dq
                if np.max(np.abs(reference-arm))>.25:continue
                for alpha in (.5,1.):
                    self.check_clearance(self.kin.matrix(arm+(reference-arm)*alpha,side),side,payload=payload)
                # Preview the Cartesian corridor, not just its endpoint.
                seed=arm.copy()
                for fraction in np.linspace(0.,1.,21):
                    seed=self.solve_pose(side,pose[:3,3]+np.asarray(offset)*fraction,pose[:3,:3],seed)
                    self.check_clearance(self.kin.matrix(seed,side),side,payload=payload,extra_margin=.010)
                for alpha in (.5,1.):
                    self.check_clearance(self.kin.matrix(seed+bias*alpha,side),side,payload=payload)
                return point,pose[:3,:3].copy()
            except Stop:continue
        raise Stop('No bounded collision-checked transport clearance step')

    @timed('motion')
    def joints(self, q14, stage, timeout=90, tolerance=.05, pose_tolerance=None):
        self.stage = stage
        q14=np.clip(q14,self.low[JOINTS]+.005,self.high[JOINTS]-.005)
        goal = self.command.copy()
        goal[JOINTS] = q14
        self.log('joint_goal',stage=stage,q=q14)
        deadline = time.monotonic()+timeout
        stable = 0
        target_pose=self.kin.matrix(q14[self.side*7:self.side*7+7],self.side) if pose_tolerance is not None else None
        while time.monotonic()<deadline:
            measured = self.measured()
            gain=.08
            correction = np.clip((q14-measured[JOINTS])*gain,-.012,.012)
            compensation=.22 if pose_tolerance is not None else .15
            goal[JOINTS] = np.clip(self.command[JOINTS]+correction,
                                  np.maximum(q14-compensation,self.low[JOINTS]+.001),
                                  np.minimum(q14+compensation,self.high[JOINTS]-.001))
            goal[JOINTS]=np.clip(goal[JOINTS],measured[JOINTS]-.22,measured[JOINTS]+.22)
            self.emit(goal)
            actual=self.measured()[JOINTS]
            err = np.max(np.abs(actual-q14))
            arrived=err<tolerance
            if target_pose is not None:
                pose=self.kin.matrix(actual[self.side*7:self.side*7+7],self.side)
                arrived &= (np.linalg.norm(pose[:3,3]-target_pose[:3,3])<pose_tolerance and
                            np.linalg.norm(Rotation.from_matrix(target_pose[:3,:3]@pose[:3,:3].T).as_rotvec())<.04)
            stable = stable+1 if arrived else 0
            if stable >= 5:
                return
        self.log('joint_timeout',stage=stage,target=q14,actual=actual,command=self.command[JOINTS])
        raise Stop(f'{stage}: joint timeout ({err:.3f} rad)')

    @timed('motion')
    def joint_path(self,q14,stage,settle=True,pose_tracking=False):
        initial=self.measured()[JOINTS]
        q14=np.asarray(q14)
        span=float(np.max(np.abs(q14-initial)))
        self.stage=stage
        moving=[np.max(np.abs(q14[side*7:side*7+7]-initial[side*7:side*7+7]))>.01 for side in (0,1)]
        # Check the complete nominal segment before streaming. emit() still
        # checks actual feedback and each filtered reference on every tick.
        for fraction in np.linspace(0,1,max(31,int(np.ceil(span/.02))+1)):
            sample=initial+(q14-initial)*fraction
            for side in (0,1):
                self.check_clearance(self.kin.matrix(sample[side*7:side*7+7],side),side,extra_margin=.005 if moving[side] else 0.)
        self.log('joint_path_start',stage=stage,q=q14,span=span,pose_tracking=pose_tracking)
        progress=0.;deadline=time.monotonic()+max(120.,span/.02+30)
        while progress<1.-1e-9 and span>1e-9:
            if time.monotonic()>deadline:raise Stop(f'{stage}: streamed path timeout')
            feedback=self.measured();measured=feedback[JOINTS]
            current={side:self.kin.matrix_and_jacobian(measured[side*7:side*7+7],side)
                     for side in (0,1) if moving[side]} if pose_tracking else {}
            candidate=min(1.,progress+.012/span)
            def admissible(fraction):
                nominal=initial+(q14-initial)*fraction
                if pose_tracking:
                    for side,(pose,_) in current.items():
                        desired=self.kin.matrix(nominal[side*7:side*7+7],side)
                        if (np.linalg.norm(desired[:3,3]-pose[:3,3])>.006 or
                                np.linalg.norm(Rotation.from_matrix(desired[:3,:3]@pose[:3,:3].T).as_rotvec())>.04):
                            return False
                    return True
                return np.max(np.abs(nominal-measured))<=.06
            low,high=progress,candidate
            for _ in range(16):
                middle=(low+high)/2
                if admissible(middle):low=middle
                else:high=middle
            if admissible(candidate):low=candidate
            progress=low
            goal=self.command.copy()
            nominal=initial+(q14-initial)*progress
            # MPC tracking bias changes with configuration and payload. Keep
            # integrating measured error while the nominal path waits for the
            # arm; a fixed initial bias can otherwise stall at the lead bound.
            if pose_tracking:
                # Follow the checked FK curve with task-space feedback. The
                # controller can change redundant joint configuration when a
                # reference saturates, while preserving the carried pose path.
                for side,(pose,jacobian) in current.items():
                    desired=self.kin.matrix(nominal[side*7:side*7+7],side)
                    error=np.r_[desired[:3,3]-pose[:3,3],
                                Rotation.from_matrix(desired[:3,:3]@pose[:3,:3].T).as_rotvec()]
                    arm_start=(0,8)[side]
                    goal[arm_start:arm_start+7]+=self._cartesian_delta(feedback,arm_start,jacobian,error,.20)
            else:
                correction=np.clip(.08*(nominal-measured),-.012,.012)
                goal[JOINTS]=np.clip(self.command[JOINTS]+correction,
                                    np.maximum(nominal-.22,self.low[JOINTS]+.001),
                                    np.minimum(nominal+.22,self.high[JOINTS]-.001))
                goal[JOINTS]=np.clip(goal[JOINTS],measured-.22,measured+.22)
            self.emit(goal,backtrack=True)
        # A following Cartesian controller can finish the task pose using
        # the remaining joints when one reference reaches its tracking bound.
        # Transit waypoints need no independent fixed-configuration settling.
        if settle:
            if pose_tracking:
                for side in (0,1):
                    if not moving[side]:continue
                    desired=self.kin.matrix(q14[side*7:side*7+7],side)
                    self.cartesian(side,desired[:3,3],desired[:3,:3],stage,
                                   position_tolerance=.006,orientation_tolerance=.04)
            else:self.joints(q14,stage,tolerance=.025,pose_tolerance=.006)
        else:
            self.log('joint_path_streamed',stage=stage,nominal=q14,measured=self.measured()[JOINTS],pose_tracking=pose_tracking)

    @timed('planning')
    def plan_empty_cartesian_return(self,side,target_q):
        """Preview an empty-hand pose corridor when joint interpolation fails."""
        if self.held or self.payload is not None:
            raise Stop('Cartesian return requires an empty hand')
        if self.command[(7,15)[side]]>=.3:
            raise Stop('Cartesian return requires an open gripper')
        start=(0,8)[side];arm=self.measured()[start:start+7]
        bias=self.command[start:start+7]-arm
        if not np.isfinite(bias).all() or np.max(np.abs(bias))>.25:
            raise Stop('Return reference bias exceeds tracking limit')
        source=self.kin.matrix(arm,side);target=self.kin.matrix(target_q,side)
        angle=np.linalg.norm(Rotation.from_matrix(target[:3,:3]@source[:3,:3].T).as_rotvec())
        segments=max(1,int(np.ceil(np.linalg.norm(target[:3,3]-source[:3,3])/.012)),
                     int(np.ceil(angle/.04)))
        # Validate the full corridor before sending its first waypoint. A
        # current bias estimate only previews feasibility; cartesian/emit
        # still enforce fresh measured and reference guards on every tick.
        samples=max(60,2*segments)
        fractions=np.unique(np.r_[np.linspace(0,1,samples+1),np.linspace(0,1,segments+1)])
        rotations=Slerp([0.,1.],Rotation.from_matrix(np.array([source[:3,:3],target[:3,:3]])))
        seed=arm.copy()
        for fraction in fractions:
            point=source[:3,3]*(1-fraction)+target[:3,3]*fraction
            q=self.solve_pose(side,point,rotations(fraction).as_matrix(),seed,reference_bias=bias)
            self.check_clearance(self.kin.matrix(q,side),side,extra_margin=.005,gripper_open=True)
            self.check_grasp_reference(side,q,bias,0.,0.)
            seed=q
        waypoints=[(source[:3,3]*(1-f)+target[:3,3]*f,rotations(f).as_matrix())
                   for f in np.linspace(0,1,segments+1)[1:]]
        self.log('empty_cartesian_return_plan',side=side,samples=len(fractions),
                 waypoints=len(waypoints),source=source,target=target)
        return waypoints

    @timed('motion')
    def initialise(self):
        # Public collection-script arm preparation poses, not object coordinates.
        self.joints(np.deg2rad([20,70,0,-35,0,0,0,20,-70,0,-35,0,0,0]),'side_lift')
        self.joints(np.deg2rad([30,10,10,-120,-60,0,0,30,-10,-10,-120,60,0,0]),'work_pose')

    def pose(self,side):
        start = (0,8)[side]
        return self.kin.matrix(self.measured()[start:start+7],side)

    def wait_stationary(self,side,timeout=10.):
        start=(0,8)[side]
        deadline=time.monotonic()+timeout
        window=[]
        while time.monotonic()<deadline:
            now=time.monotonic()
            window.append((now,self.measured()[start:start+7].copy()))
            window=[sample for sample in window if now-sample[0]<=.7]
            if now-window[0][0]>=.5 and np.max(np.ptp([q for _,q in window],axis=0))<.003:
                return
            time.sleep(.1)
        raise Stop('Arm did not settle before payload measurement')

    @timed('planning')
    def solve_pose(self, side, xyz, rotation, seed=None, joint_margin=.003,reference_bias=None):
        start=(0,8)[side]
        if seed is None: seed=self.measured()[start:start+7]
        limits=self.kin.limits[side]
        lower,upper=feedback_joint_bounds(limits,np.asarray(seed),joint_margin,reference_bias)
        cache={}
        def evaluate(q):
            key=tuple(q)
            if cache.get('key')!=key:
                mat,jac=self.kin.matrix_and_jacobian(q,side)
                error=Rotation.from_matrix(rotation@mat[:3,:3].T).as_rotvec()
                cache.update(key=key,residual=np.r_[mat[:3,3]-xyz,.15*error,.0001*(q-seed)],
                             jacobian=np.vstack((jac[:3],-.15*rotation_log_right_inverse(error)@jac[3:],.0001*np.eye(7))))
            return cache
        def residual(q):return evaluate(q)['residual']
        def derivative(q):return evaluate(q)['jacobian']
        best=None
        position_limit,rotation_limit=(.0005,.001) if reference_bias is not None else (.005,.08)
        public_place=np.deg2rad([-30,10 if side==0 else -10,10 if side==0 else -10,-80,-70 if side==0 else 70,0,0])
        for initial in (seed,public_place,(lower+upper)/2):
            out=least_squares(residual,np.clip(initial,lower+1e-5,upper-1e-5),jac=derivative,bounds=(lower,upper),max_nfev=160)
            error=residual(out.x)
            if np.linalg.norm(error[:3])<position_limit and np.linalg.norm(error[3:6])/.15<rotation_limit:
                score=np.linalg.norm(out.x-seed)
                if score<.15:return out.x
                if best is None or score<best[0]: best=(score,out.x)
        if best is None: raise Stop('No bounded IK for requested pose')
        return best[1]

    @timed('planning')
    def transfer_pose(self,side,part,source,rotation,seed=None):
        above=np.array(source).copy()
        above[2]=max(b.rim for b in self.bins.values())+(.055 if part.kind=='A' else .11)
        for forward in ((0.,.08,.12) if part.kind in ('A','C') else (0.,)):
            point=above.copy();point[0]+=forward
            # The pitch wrist needs positive MPC compensation when loaded;
            # reserving the roll/yaw wrists as well excludes reachable grasps.
            reserve=A_LIFT_JOINT_MARGIN if part.kind=='A' else .003
            try: return point,self.solve_pose(side,point,rotation,seed,joint_margin=reserve)
            except Stop: pass
        raise Stop('No bounded transfer pose above the bin rims')

    @timed('planning')
    def observation_hover(self,side,part,hover):
        """Reach a checked observation pose before refining the table grasp."""
        measured=self.measured();start=(0,8)[side];q=measured[start:start+7]
        current=self.kin.matrix(q,side)[:3,:3]
        axis=np.asarray(part.axis)
        nominal=np.array([[axis[1],axis[0],0],[-axis[0],axis[1],0],[0,0,1]])
        rotations=[nominal@Rotation.from_euler('z',yaw).as_matrix()@
                   Rotation.from_euler('x',tilt).as_matrix()
                   for yaw in np.arange(0,2*np.pi,np.pi/6) for tilt in (0.,-.26,.26,-.52,.52,-.78,.78)]
        rotations.sort(key=lambda rot:np.linalg.norm(Rotation.from_matrix(rot@current.T).as_rotvec()))
        for rotation in [current]+rotations:
            try:
                above=self.solve_pose(side,hover,rotation,q)
                for fraction in np.linspace(0,1,max(31,int(np.ceil(np.max(np.abs(above-q))/.02))+1)):
                    self.check_clearance(self.kin.matrix(q+(above-q)*fraction,side),side,extra_margin=.005)
                break
            except Stop:continue
        else:raise Stop('No checked observation hover')
        self.log('observation_hover_plan',kind=part.kind,side=side,point=hover,rotation=rotation,q=above)
        goal=measured[JOINTS];goal[side*7:side*7+7]=above
        self.joint_path(goal,'observation_hover')
        return rotation,np.asarray(part.center).copy()

    def check_grasp_reference(self, side, grasp, reference_bias, margin, table_reserve):
        reference=grasp+reference_bias
        limits=self.kin.limits[side]
        if np.any(reference<limits[:,0]+.001) or np.any(reference>limits[:,1]-.001):
            raise Stop('Grasp reference exceeds joint limits')
        for alpha in (.5,1.):
            self.check_clearance(self.kin.matrix(grasp+reference_bias*alpha,side),side,
                                 extra_margin=margin,table_reserve=table_reserve)

    @timed('planning')
    def orient_for_grasp(self,side,part,hover,validator=None,height_adjust=0.,c_tilt_stage=0,a_tilt_stage=0,execute=True,a_yaw_refinement=False):
        start=(0,8)[side]
        q=self.measured()[start:start+7]
        reference_bias=self.command[start:start+7]-q
        axis=np.array(part.axis)
        nominal=np.array([[axis[1],axis[0],0],[-axis[0],axis[1],0],[0,0,1]])
        options=[]
        yaws=np.arange(0,2*np.pi,np.pi/6)
        if part.kind=='A' and height_adjust==0. and a_tilt_stage==0:
            self.log('grasp_search_start',kind='A',side=side,q=q,reference_bias=reference_bias,
                     fine_yaw=a_yaw_refinement)
        if part.kind=='A' and a_yaw_refinement:
            # Additional headings remain available after the established
            # coarse search. A35's early fine search selected a deep flat
            # grasp which could not finish under the changing MPC bias; the
            # original search had already completed tilted A grasps in A28.
            yaws=np.sort(np.r_[yaws+np.deg2rad(5),yaws-np.deg2rad(5)])%(2*np.pi)
        if part.kind=='C':yaws=np.deg2rad([0,10,-10,14,-14,180,170,190,166,194])
        failures={}
        tilts=(0.,-.26,.26,-.52,.52,-.78,.78) if part.kind=='A' else ((0.,-.26,.26,-.52,.52) if part.kind=='C' else (0.,-.26,.26))
        offsets=[(0.,0.)] if part.kind!='A' else [(0.,0.),(-part.extent[0]*.35,0.),(part.extent[0]*.35,0.),(0.,.012),(0.,-.012),(.015,0.),(-.015,0.)]
        if part.kind=='C':
            shaft=float(np.sign(np.dot(np.asarray(part.full_center or part.center)[:2]-np.asarray(part.center)[:2],axis)))
            # C20 still pivoted in the 10mm pinch. First test a grip nearer
            # the shaft, reducing the full observed envelope lever arm. Keep
            # the established offsets as fallbacks; every grip still needs
            # complete lift, transport and release validation.
            balanced=(.025,.020) if any(p.kind=='C' for p in getattr(self,'settled_parts',[])) else (.025,)
            offsets=[(shaft*v,0.) for v in (*balanced,.010,.005,0.,.015,-.015,-.025)] if shaft else [(0.,0.)]
        grasp_margin=.002 if part.kind=='A' else 0.
        table_reserve=.003 if part.kind=='A' else 0.
        for offset_index,(along,across) in enumerate(offsets):
            grasp_point=np.array(part.center)
            grasp_point[:2]+=axis*along+np.array([-axis[1],axis[0]])*across
            if part.kind=='A':grasp_point[2]=max(part.center[2]+.004,self.table+.013)+height_adjust
            # Seat the handle inside the pads instead of pinching its upper
            # edge. The observed geometry still has to clear the tabletop.
            if part.kind=='C':grasp_point[2]=max(grasp_point[2],self.table+.015)
            above_point=hover.copy();above_point[:2]=grasp_point[:2]
            tiers=[(0.,-.26,.26)]
            if part.kind=='A':tiers=[((0.,-.13,.13,-.26,.26),(-.52,.52),(-.65,.65),(-.78,.78))[a_tilt_stage]]
            if part.kind=='C':
                c_tiers=[(.52,-.52,.65,-.65),(.78,-.78),(1.05,-1.05),(0.,-.26,.26)]
                tiers=[c_tiers[c_tilt_stage]]
            for tier in tiers:
                rotations=[(yaw,tilt) for yaw in yaws for tilt in tier]
                for yaw,tilt in rotations:
                    rot=nominal@Rotation.from_euler('z',yaw).as_matrix()@Rotation.from_euler('x',tilt).as_matrix()
                    phase='clearance_prefilter'
                    try:
                        nominal_pose=np.eye(4);nominal_pose[:3,:3]=rot;nominal_pose[:3,3]=grasp_point
                        if grasp_margin:self.check_clearance(nominal_pose,side,extra_margin=grasp_margin,table_reserve=table_reserve)
                        phase='grasp'
                        grasp=self.solve_pose(side,grasp_point,rot,q,
                                              reference_bias=reference_bias if part.kind=='A' else None)
                        phase='hover'
                        above=self.solve_pose(side,above_point,rot,grasp)
                        phase='high'
                        if part.kind in ('A','C'):
                            clear=grasp_point.copy();clear[2]=self.table+.10
                            self.solve_pose(side,clear,rot,above,joint_margin=A_LIFT_JOINT_MARGIN)
                        else:self.transfer_pose(side,part,grasp_point,rot,above)
                        phase='clearance'
                        self.check_clearance(self.kin.matrix(grasp,side),side,extra_margin=grasp_margin,table_reserve=table_reserve)
                        if part.kind=='A':
                            # The nominal grasp can be clear while the MPC
                            # compensation drives an open finger into the
                            # target. Reject it before descent, using the
                            # observed bias without weakening the live guard.
                            phase='grasp_reference'
                            self.check_grasp_reference(side,grasp,reference_bias,grasp_margin,table_reserve)
                        deviation=min(abs(yaw),abs(yaw-np.pi),abs(yaw-2*np.pi))
                        options.append((np.linalg.norm(above-q)+np.linalg.norm(grasp-above)+deviation*2,rot,above,grasp_point.copy()))
                    except Stop as e:
                        key=phase+': '+str(e);failures[key]=failures.get(key,0)+1
                if options and validator is not None:
                    accepted=[]
                    for option in sorted(options,key=lambda x:x[0]):
                        try:
                            validator(option[1],option[3])
                            accepted=[option]
                            break
                        except Stop as e:self.log('grasp_route_rejected',kind=part.kind,reason=str(e))
                    options=accepted
                if options:break
            if options:break
        if not options:
            self.log('grasp_rejected',kind=part.kind,side=side,height_adjust=height_adjust,reasons=failures)
            if part.kind=='A' and height_adjust<.008:
                return self.orient_for_grasp(side,part,hover,validator,round(min(.008,height_adjust+.001),6),
                                             a_tilt_stage=a_tilt_stage,execute=execute,a_yaw_refinement=a_yaw_refinement)
            if part.kind=='A' and a_tilt_stage<3:
                return self.orient_for_grasp(side,part,hover,validator,0.,a_tilt_stage=a_tilt_stage+1,
                                             execute=execute,a_yaw_refinement=a_yaw_refinement)
            if part.kind=='A' and not a_yaw_refinement:
                return self.orient_for_grasp(side,part,hover,validator,0.,execute=execute,a_yaw_refinement=True)
            if part.kind=='C' and c_tilt_stage<3:
                return self.orient_for_grasp(side,part,hover,validator,height_adjust,c_tilt_stage+1,execute=execute)
            raise Stop('No collision-checked grasp/hover pose within physical limits')
        _,rot,above,grasp_point=min(options,key=lambda x:x[0])
        self.log('grasp_plan',kind=part.kind,side=side,rotation=rot,hover_q=above,grasp_point=grasp_point)
        goal=self.measured()[JOINTS]
        goal[side*7:side*7+7]=above
        if execute:self.joints(goal,'hover_configuration')
        return rot,grasp_point

    def _cartesian_delta(self,state,start,jacobian,error,gain):
        weight=np.diag([1,1,1,.16,.16,.16])
        j=weight@jacobian
        lower=np.maximum(-.012,self.low[start:start+7]+.0001-self.command[start:start+7])
        upper=np.minimum(.012,self.high[start:start+7]-.0001-self.command[start:start+7])
        reference=state[start:start+7]-self.command[start:start+7]
        # Near the compensation limit, more reference motion is not evidence
        # of actuator motion. Solve the pose correction with the other joints
        # until feedback brings this joint back inside the reserved window.
        j[:,np.abs(reference)>.215]=0.
        lower=np.maximum(lower,np.minimum(0.,reference-.22))
        upper=np.minimum(upper,np.maximum(0.,reference+.22))
        return lsq_linear(np.vstack((j,.02*np.eye(7))),np.r_[weight@error*gain,np.zeros(7)],
                          bounds=(lower,upper),tol=1e-7,max_iter=50).x

    @timed('motion')
    def cartesian(self,side,xyz,rotation,stage,timeout=90,position_tolerance=.007,settle_frames=5,orientation_tolerance=.10):
        if settle_frames not in (1,5):raise ValueError('Cartesian settling requires one transit frame or five endpoint frames')
        if not 0<orientation_tolerance<=.10:raise ValueError('Orientation tolerance must preserve the existing limit')
        self.stage = stage
        start = (0,8)[side]
        xyz,rotation = np.asarray(xyz),np.asarray(rotation)
        deadline = time.monotonic()+timeout
        stable = 0
        while time.monotonic()<deadline:
            q = self.measured()
            mat,jac = self.kin.matrix_and_jacobian(q[start:start+7],side)
            error = np.r_[xyz-mat[:3,3],Rotation.from_matrix(rotation@mat[:3,:3].T).as_rotvec()]
            stable = stable+1 if np.linalg.norm(error[:3])<position_tolerance and np.linalg.norm(error[3:])<orientation_tolerance else 0
            if stable>=settle_frames:
                self.log('arrived',stage=stage,side=side,position=mat[:3,3],error=error)
                return
            # Integral Cartesian correction of the pending reference handles
            # static MPC tracking bias; pending error and joint slew remain bounded.
            # Use the transport controller's compensation window inside the
            # bounded solve, so a saturated joint leaves correction to the
            # other joints. A reference already outside that window can only
            # move inward; emit() retains the independent 0.25 rad guard.
            gain=.08 if stage in ('descend','lift') and mat[2,3]<self.table+.06 else .20
            dq=self._cartesian_delta(q,start,jac,error,gain)
            goal = self.command.copy()
            goal[start:start+7] += dq
            self.emit(goal,backtrack=True)
        raise Stop(f'{stage}: Cartesian timeout, position {np.linalg.norm(error[:3]):.4f}m, orientation {np.linalg.norm(error[3:]):.3f}rad')

    def refined_grasp_height(self,side,source,rotation,state,minimum_height):
        """Recheck the final A grasp with the current, changing MPC bias.

        A27's initial hover bias was safe, but the bias changed in the last
        7mm of descent. A small upward correction can preserve pad engagement
        before the reference reaches the table and feedback drifts sideways.
        Never lower the selected height or chase a correction above the part.
        """
        start=(0,8)[side];arm=state[start:start+7]
        bias=self.command[start:start+7]-arm
        # Keep the same 8mm search range as the initial A planner, but let
        # the observed top determine the usable part of that range. A31 had
        # a checked 4mm correction with >1mm TCP engagement; the old fixed
        # 3mm cap discarded it and repeated three unsuccessful retreats.
        for rise in np.arange(0.,.0081,.001):
            point=np.asarray(source).copy();point[2]+=rise
            if point[2]<minimum_height-1e-9:continue
            # Preserve a separately checked nominal grasp; its pad geometry
            # may allow the TCP close to the top of an irregular bracket.
            # Additional height is only available below the observed top.
            if rise>0. and point[2]>self.target.top-.001:continue
            try:
                q=self.solve_pose(side,point,rotation,arm,reference_bias=bias)
                self.check_clearance(self.kin.matrix(q,side),side,extra_margin=.002,table_reserve=.003)
                self.check_grasp_reference(side,q,bias,.002,.003)
                return point
            except Stop:continue
        raise Stop('No engaged A grasp height with current reference clearance')

    def retreat_for_grasp_refinement(self,side):
        """Gain 12mm of clearance with the open hand before local replanning."""
        state=self.measured();start=(0,8)[side];arm=state[start:start+7]
        pose,jac=self.kin.matrix_and_jacobian(arm,side)
        bias=self.command[start:start+7]-arm;delta=np.array([0.,0.,.012])
        self.check_clearance(pose,side)
        step=self._cartesian_delta(state,start,jac,np.r_[delta,[0.,0.,0.]],.20)
        reference=self.command[start:start+7]+step
        for fraction in (.5,1.):
            self.check_clearance(self.kin.matrix(arm+(reference-arm)*fraction,side),side)
        seed=arm.copy()
        for fraction in np.linspace(0,1,17):
            seed=self.solve_pose(side,pose[:3,3]+delta*fraction,pose[:3,:3],seed,reference_bias=bias)
            self.check_clearance(self.kin.matrix(seed,side),side)
            self.check_grasp_reference(side,seed,bias,0.,0.)
        self.log('grasp_refinement_retreat',side=side,goal=pose[:3,3]+delta)
        self.cartesian(side,pose[:3,3]+delta,pose[:3,:3],'grasp_refine_retreat',
                       position_tolerance=.002,orientation_tolerance=.02)

    def refined_grasp_pose(self,side,source,rotation,state,minimum_height,rejected=(),reference_biases=()):
        """Bounded A grip refinement with current and failed-descent references."""
        start=(0,8)[side];arm=state[start:start+7]
        actual=self.kin.matrix(arm,side);bias=self.command[start:start+7]-arm
        self.check_clearance(actual,side)
        reference_biases=tuple(np.asarray(b) for b in reference_biases)
        if any(b.shape!=(7,) or not np.isfinite(b).all() or np.max(np.abs(b))>.25
               for b in reference_biases):
            raise Stop('Invalid failed-descent reference bias')
        # Retreat changes the MPC bias. A33 repeatedly accepted an attitude
        # under that transient bias which failed again near the table. Keep
        # the observed failure biases as endpoint constraints, and allow a
        # small shift along/across the visible part instead of rotating about
        # one fixed point indefinitely. Every candidate retains full geometry
        # checks and a checked approach under the current reference.
        offsets=[(0.,0.)]
        if reference_biases:
            offsets.extend((a,b) for distance in (.003,.006)
                           for a,b in ((distance,0.),(-distance,0.),(0.,distance),(0.,-distance)))
        part_axis=np.asarray(self.target.axis);across_axis=np.array([-part_axis[1],part_axis[0]])
        options=[]
        for rise in np.arange(0.,.0081,.001):
            for along,across in offsets:
                point=np.asarray(source).copy();point[2]+=rise
                point[:2]+=part_axis*along+across_axis*across
                if point[2]<minimum_height-1e-9 or point[2]>max(source[2],self.target.top-.001):continue
                for angle in (.025,-.025,.05,-.05,.075,-.075,.10,-.10):
                    for axis in ('x','y','z'):
                        rot=rotation@Rotation.from_euler(axis,angle).as_matrix()
                        if any(np.linalg.norm(Rotation.from_matrix(rot@old.T).as_rotvec())<.012 for old in rejected):continue
                        turn=np.linalg.norm(Rotation.from_matrix(rot@actual[:3,:3].T).as_rotvec())
                        score=np.linalg.norm(point-actual[:3,3])+.1*turn+.5*rise+.2*np.hypot(along,across)
                        options.append((score,point,rot))
        for _,point,rot in sorted(options,key=lambda item:item[0]):
            try:
                nominal=np.eye(4);nominal[:3,:3]=rot;nominal[:3,3]=point
                self.check_clearance(nominal,side,extra_margin=.002,table_reserve=.003)
                q=self.solve_pose(side,point,rot,arm,reference_bias=bias)
                self.check_clearance(self.kin.matrix(q,side),side,extra_margin=.002,table_reserve=.003)
                for observed_bias in (bias,*reference_biases):
                    self.check_grasp_reference(side,q,observed_bias,.002,.003)
                orientations=Slerp([0.,1.],Rotation.from_matrix(np.array([actual[:3,:3],rot])))
                seed=arm.copy()
                for fraction in np.linspace(0,1,17):
                    pose=np.eye(4);pose[:3,:3]=orientations(fraction).as_matrix()
                    pose[:3,3]=actual[:3,3]*(1-fraction)+point*fraction
                    self.check_clearance(pose,side)
                    seed=self.solve_pose(side,pose[:3,3],pose[:3,:3],seed,reference_bias=bias)
                    self.check_grasp_reference(side,seed,bias,0.,0.)
                return point,rot
            except Stop:continue
        raise Stop('No checked local A grasp attitude refinement')

    def grasp_ik_step(self,side,point,rotation,state):
        """Follow a nearby checked IK solution without the damped-Jacobian drift.

        A32 approached the height limit while drifting sideways. Its bounded
        IK already supplied a small shoulder/elbow correction that the damped
        differential solve suppressed. Keep the correction on that joint-space
        line, with the same slew and compensation bounds as Cartesian control.
        """
        start=(0,8)[side];arm=state[start:start+7]
        bias=self.command[start:start+7]-arm
        q=self.solve_pose(side,point,rotation,arm,reference_bias=bias)
        self.check_clearance(self.kin.matrix(q,side),side,extra_margin=.002,table_reserve=.003)
        self.check_grasp_reference(side,q,bias,.002,.003)
        difference=q-arm
        lower=np.maximum(-.012,self.low[start:start+7]+.0001-self.command[start:start+7])
        upper=np.minimum(.012,self.high[start:start+7]-.0001-self.command[start:start+7])
        lower=np.maximum(lower,np.minimum(0.,-bias-.22))
        upper=np.minimum(upper,np.maximum(0.,-bias+.22))
        scale=.20
        for delta,lo,hi in zip(difference,lower,upper):
            if delta>1e-12:scale=min(scale,hi/delta)
            elif delta<-1e-12:scale=min(scale,lo/delta)
        if scale<=0.:raise Stop('No bounded step toward checked grasp IK')
        step=difference*scale
        for fraction in (.5,1.):
            sample=arm+step*fraction
            self.check_clearance(self.kin.matrix(sample,side),side)
            self.check_grasp_reference(side,sample,bias,0.,0.)
        self.log('grasp_ik_step',side=side,scale=scale,delta=step)
        return step

    @timed('motion')
    def finish_grasp_descent(self,side,source,rotation,timeout=90):
        """Close the last A descent under fresh reference and pose checks."""
        self.stage='descend';start=(0,8)[side]
        source=np.asarray(source);rotation=np.asarray(rotation)
        anchor=rotation.copy();anchor_source=source.copy();rejected=[];failed_biases=[]
        goal=source.copy();stable=0;deadline=time.monotonic()+timeout;approaching=False
        while time.monotonic()<deadline:
            state=self.measured()
            pose,jac=self.kin.matrix_and_jacobian(state[start:start+7],side)
            try:
                if approaching and np.linalg.norm(goal-pose[:3,3])>.005:
                    # The checked recovery sweep starts >12mm from its goal.
                    # A34 rejected that endpoint using transient moving-arm
                    # bias before reaching it. Transit still checks the actual
                    # hand and every bounded reference in emit(); terminal
                    # geometry must pass again within the last 5mm, and always
                    # before any arrival/closure can be accepted.
                    checked=goal
                else:
                    approaching=False
                    checked=self.refined_grasp_height(side,source,rotation,state,goal[2])
            except Stop:
                if len(rejected)>=3:raise
                failed_biases.append(self.command[start:start+7]-state[start:start+7])
                self.retreat_for_grasp_refinement(side)
                rejected.append(rotation.copy())
                state=self.measured()
                goal,rotation=self.refined_grasp_pose(side,anchor_source,anchor,state,goal[2],rejected,
                                                       reference_biases=failed_biases)
                source=source.copy();source[:2]=goal[:2]
                self.stage='descend';stable=0;approaching=True
                self.log('grasp_pose_refined',kind='A',goal=goal,rotation=rotation,attempt=len(rejected),
                         reference_biases=failed_biases)
                continue
            if checked[2]>goal[2]+1e-9:
                self.log('grasp_height_refined',kind='A',source=source,goal=checked,
                         rise=float(checked[2]-source[2]))
                goal=checked;stable=0
            self.check_clearance(pose,side)
            error=np.r_[goal-pose[:3,3],Rotation.from_matrix(rotation@pose[:3,:3].T).as_rotvec()]
            arrived=(np.linalg.norm(error[:3])<.002 and np.linalg.norm(error[3:])<.02
                     and pose[2,3]<=max(source[2]+.002,self.target.top-.001))
            stable=stable+1 if arrived else 0
            if stable>=5:
                self.log('arrived',stage=self.stage,side=side,position=pose[:3,3],error=error)
                return goal,rotation
            command=self.command.copy()
            if np.linalg.norm(error[:3])<.005 and np.linalg.norm(error[3:])<.05:
                try:step=self.grasp_ik_step(side,goal,rotation,state)
                except Stop as ex:
                    self.log('grasp_ik_rejected',reason=str(ex))
                    step=self._cartesian_delta(state,start,jac,error,.08)
                command[start:start+7]+=step
            else:
                command[start:start+7]+=self._cartesian_delta(state,start,jac,error,.08)
            self.emit(command,backtrack=True)
        raise Stop('A final grasp descent did not settle with fresh clearance')

    @timed('motion')
    def gripper(self,side,close,stage,contact_stop=False,release_step=.04):
        handover=getattr(self,'handover',None)
        if not close and handover is not None:
            handover.authorize_open(side)
            if side==handover.donor:handover.begin_release()
        self.stage = stage
        index = (7,15)[side]
        target = 1. if close else 0.
        step=.04 if close else release_step
        history=[]
        for _ in range(35):
            goal = self.command.copy()
            goal[index] += np.clip(target-goal[index],-step,step)
            self.emit(goal)
            if close and contact_stop:
                feedback=self.measured()[index]
                history.append(feedback)
                if (len(history)>=3 and .05<feedback<.95 and self.command[index]-feedback>.12
                        and max(history[-3:])-min(history[-3:])<.02):
                    target=min(1.,feedback+.08)
                    for _ in range(5):
                        goal=self.command.copy()
                        goal[index]+=np.clip(target-goal[index],-.04,.04)
                        self.emit(goal)
                    self.log('gripper_contact_stop',side=side,feedback=self.measured()[index],command=self.command[index])
                    break
        self.log('gripper',side=side,close=close,feedback=self.measured()[index])

    @timed('motion')
    def restore_loaded_gripper(self,side,recorded_command):
        index=(7,15)[side]
        measured=self.measured()[index]
        if not np.isfinite(recorded_command) or not measured<=recorded_command<=1.:
            raise Stop('Checkpoint gripper command would reduce held contact')
        # Contact feedback is an obstructed jaw position, not the closing
        # reference. Preserve the checkpoint preload through the normal filter.
        goal=self.command.copy();goal[index]=recorded_command
        self.stage='resume_hold'
        for _ in range(4):self.emit(goal)
        self.log('gripper_preload_restored',side=side,feedback=self.measured()[index],command=recorded_command)

    def close(self):
        for sub in self.subs:
            sub.unregister()
        self.env.close()
