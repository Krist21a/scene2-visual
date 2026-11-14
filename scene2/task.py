"""Explicit perception/grasp/place state machine and visual completion ledger."""
import time
from .timing import timed
from dataclasses import asdict,replace
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from .control import Stop,A_LIFT_JOINT_MARGIN
from .perception import Detector, masks, point_image, serialise
from .settling import release_center, settling_fits, settled_corners, held_fits, c_profile, support_bias
from .release import free_drop_clear, side_opening_clear, C_OPENING_PLANNING_LIMIT
from .payload import source_payload

C_RELEASE_MARGIN=.010
C_RELEASE_RESERVE=.002
C_OCCUPIED_SETTLING_MARGIN=.005
RELEASE_JOINT_MARGIN=np.r_[np.full(4,.003),np.full(3,.08)]
C_ELBOW_TRANSIT_MARGIN=.30
C_ELBOW_IK_MARGIN=.10

class EmptyGrasp(Stop):
    """Table target still visible and jaws fully closed without an object."""


def fits_bin(part,bin,margin=.003):
    if part.kind in ('B','C') and part.footprint:
        # The complete observed hull includes every supported extreme. A
        # rotated PCA rectangle adds unoccupied corners to irregular B parts
        # and can alternate across the margin despite a clear actual contour.
        footprint=np.asarray(part.footprint)
        if bin.cavity is not None:
            return all(bin.contains(point,margin) for point in footprint)
        return bin.contains(footprint.min(0),margin) and bin.contains(footprint.max(0),margin)
    axis=np.asarray(part.axis)
    basis=np.column_stack((axis,[-axis[1],axis[0]]))
    half=np.abs(basis)@np.asarray(part.full_extent or part.extent)/2
    center=np.array(part.full_center or part.center)[:2]
    return bin.contains(center-half,margin) and bin.contains(center+half,margin)


def checkpoint_counts(rows):
    initial=next(e for e in reversed(rows) if e['event'] in ('initial_occupancy','held_transport_resumed'))
    counts=dict(initial.get('counts',initial.get('initial_counts')))
    ledger=next((e['ledger'] for e in reversed(rows) if e['event']=='completed'),[])
    for entry in ledger:counts[entry['kind']]+=1
    return counts, bool(ledger)


class Sorter:
    def __init__(self,robot,scene,log,snapshot_writer):
        from .wrist import WristScene
        self.robot,self.scene,self.log,self.save = robot,scene,log,snapshot_writer
        self.wrist=WristScene(robot.rospy,scene,robot.kin)
        self.detector = Detector()
        self.completed = []
        self.initial_counts = dict.fromkeys('ABC',0)
        self.bin_parts={k:[] for k in 'ABC'}
        self.last_stamp = None
        self.observer_return=None

    @timed('perception')
    def observe(self, label, required_bins=None):
        if required_bins is None: required_bins=set('ABC')
        deadline = time.monotonic()+10
        last_error = None
        while time.monotonic()<deadline:
            try:
                s = self.scene.read(candidate_stride=2)
                if self.last_stamp is not None and s['stamp'] <= self.last_stamp:
                    raise ValueError('Repeated/out-of-order camera frame')
                parts,bins = self.detector.detect(s)
                views=[s]
                for side in (0,1):
                    try:
                        ws=self.wrist.read(side,s['table_z_base_m'])
                        views.append(ws)
                        wp,wb=self.detector.detect(ws)
                        self.save(f'{label}-wrist{side}',ws,wp,wb)
                        for p in wp:
                            # A wrist may see a placed part while its bin floor
                            # is outside that camera's view. Apply the head's
                            # same source-region exclusion before merging.
                            # Placed geometry remains in settled_parts.
                            if any(b.contains(p.center[:2],-.025) for b in bins.values()):continue
                            matches=[i for i,other in enumerate(parts) if p.kind==other.kind and np.linalg.norm(np.array(p.center)-other.center)<.025]
                            if not matches:
                                parts.append(p)
                            elif p.kind=='A':
                                i=matches[0]
                                if np.prod(p.extent)>np.prod(parts[i].extent)*1.15:parts[i]=p
                    except ValueError as e:
                        self.log('wrist_unavailable',side=side,reason=str(e))
                self.last_stamp = s['stamp']
                if not set(required_bins).issubset(bins):
                    raise ValueError(f'Required bin floors not visible: {set(required_bins)-set(bins)}')
                self.robot.table = s['table_z_base_m']
                self.robot.bins = bins
                # Motion guards retain their conservative visible-floor walls.
                # Settling uses the same fitted cavity as final verification.
                self.release_bins=dict(bins)
                if 'C' in bins:
                    cavity=self.tracked_release_cavity(s,bins)
                    self.release_bins['C']=cavity
                    self.log('release_cavity',bin=asdict(cavity))
                self.robot.parts = parts
                self.robot.settled_parts=[p for values in self.bin_parts.values() for p in values]
                self.views=views
                self.save(label,s,parts,bins)
                self.log('observation',label=label,**serialise(parts,bins))
                return s,parts,bins
            except (ValueError,RuntimeError) as e:
                last_error = e
                time.sleep(.2)
        raise Stop(f'Observation failed: {last_error}')

    def tracked_release_cavity(self,snapshot,bins):
        from .bin_geometry import cavity_bounds,propagate_cavity,register_cavity
        colour=masks(snapshot['rgb'])[1]['C']
        # Box dimensions are rigid. Even a formally valid unconstrained fit
        # can trade width against wall height in a changed view (C13), losing
        # several millimetres. Prefer the known-empty dimensions only when
        # the current colour/depth and independent bin references validate
        # their registration; otherwise use the ordinary fit/fallback.
        reference=getattr(self,'release_empty_cavity_reference',None)
        if reference is not None:
            registered=register_cavity(snapshot,colour,bins['C'],*reference,bins)
            if registered is not None:
                cavity,details=registered
                self.release_cavity_reference=(cavity,dict(bins))
                self.log('release_cavity_registered',stamp=cavity.stamp,**details)
                return cavity
        cavity=cavity_bounds(snapshot,colour,bins['C'])
        if cavity.cavity is not None:
            self.release_cavity_reference=(cavity,dict(bins))
        else:
            # A fitted silhouette after a placement can be shortened by the
            # occupied bin. Keep the known-empty observation as the first
            # anchor; each propagation independently checks the other bins
            # and visible floor and applies its registration uncertainty inset.
            for name in ('release_empty_cavity_reference','release_cavity_reference'):
                reference=getattr(self,name,None)
                if reference is None:continue
                propagated=propagate_cavity(*reference,bins)
                if propagated is None:continue
                cavity=propagated
                self.log('release_cavity_propagated',reference_stamp=reference[0].stamp,
                         reference_kind=name,stamp=cavity.stamp)
                break
        return cavity

    def close(self):
        self.wrist.close()

    def partial_target(self,part,source,old_table):
        if part.kind not in ('A','C') or not part.occupied_xy:return None
        from scipy.spatial import cKDTree
        shift=source-np.array(part.center)
        shift[2]=self.robot.table-old_table
        xy=np.array(part.occupied_xy)+shift[:2]
        tree=cKDTree(xy)
        count=0
        residuals=[]
        for view in self.views:
            points,valid=point_image(view);colours,_=masks(view['rgb'])
            region=valid&colours[part.kind]&(points[:,:,2]>self.robot.table+.004)&(points[:,:,2]<self.robot.table+.045)
            axis=np.array(part.axis);basis=np.column_stack((axis,[-axis[1],axis[0]]))
            local=(points[:,:,:2]-(np.array(part.center)+shift)[:2])@basis
            region&=np.all(np.abs(local)<np.array(part.extent)/2+.010,axis=-1)
            observed=points[region]
            if not len(observed):continue
            distance,indices=tree.query(observed[:,:2])
            matched=distance<.007
            if matched.sum()>=15 and matched.mean()>.8:
                count+=int(matched.sum())
                residuals.append(observed[matched,:2]-xy[indices[matched]])
        if count<15:return None
        correction=np.clip(np.median(np.vstack(residuals),axis=0),-.004,.004)
        shift[:2]+=correction
        xy+=correction
        refined=replace(part,center=(np.array(part.center)+shift).tolist(),
                        full_center=(np.array(part.full_center or part.center)+shift).tolist(),
                        top=part.top+shift[2],stamp=self.last_stamp,
                        footprint=(np.array(part.footprint)+shift[:2]).tolist(),occupied_xy=xy.tolist())
        self.log('partial_target_confirmed',kind=part.kind,points=count,shift=shift)
        # Occlusion can rotate a fragment's PCA axis and move its fitted
        # centre beyond the centroid matching radius (C26). Associate using
        # the actual visible handle support, not that unstable fitted box.
        # Require dense, almost entirely coincident support; nearby objects
        # and fragments without sufficient depth evidence remain obstacles.
        tree=cKDTree(xy)
        for observed in list(self.robot.parts):
            support=np.asarray(observed.occupied_xy or []).reshape(-1,2)
            if (observed.kind!=part.kind or len(support)<30 or
                    abs(observed.top-refined.top)>.008):continue
            distance,_=tree.query(support)
            fraction=float(np.mean(distance<.004))
            if fraction<.95:continue
            self.robot.parts=[p for p in self.robot.parts if p is not observed]
            self.log('target_fragment_associated',kind=part.kind,points=len(support),
                     supported_fraction=fraction,center=observed.center)
        self.robot.parts.append(refined)
        return refined

    @timed('verification')
    def confirm_held(self,part,side,original):
        robot = self.robot
        stable = 0
        empty = 0
        last_snapshot=None;last_wrist=None
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            try:s = self.scene.read()
            except (ValueError,RuntimeError) as e:
                self.log('held_frame_retry',reason=str(e))
                time.sleep(.1)
                continue
            if s['stamp'] == self.last_stamp:
                time.sleep(.1)
                continue
            self.last_stamp = s['stamp']
            points,valid = point_image(s)
            colours,_ = masks(s['rgb'])
            tcp = robot.pose(side)[:3,3]
            near = np.linalg.norm(points-tcp,axis=-1)<.065
            elevated = points[:,:,2] > robot.table+.055
            visible = valid & colours[part.kind] & near & elevated
            visible_count=int(visible.sum())
            held_view='head';wrist_snapshot=None
            if visible_count<12:
                try:
                    wrist_snapshot=self.wrist.read(side,robot.table)
                    wp,wvalid=point_image(wrist_snapshot);wcolours,_=masks(wrist_snapshot['rgb'])
                    wrist_count=int((wvalid&wcolours[part.kind]&(np.linalg.norm(wp-tcp,axis=-1)<.065)&(wp[:,:,2]>robot.table+.055)).sum())
                    if wrist_count>visible_count:visible_count=wrist_count;held_view='wrist'
                except ValueError:pass
            current,_ = self.detector.detect(s)
            last_snapshot=s;last_wrist=wrist_snapshot
            remains = any(p.kind==part.kind and np.linalg.norm(np.array(p.center)[:2]-original[:2])<.035 for p in current)
            grip = robot.measured()[(7,15)[side]]
            ok = visible_count >= 12 and not remains and .02 < grip < .97
            stable = stable+1 if ok else 0
            empty=empty+1 if remains and grip>=.97 and visible_count<12 else 0
            self.log('held_check',visible_points=visible_count,view=held_view,remains_at_source=remains,gripper=grip,stable=stable)
            if stable>=3:
                robot.retained=True
                self.save('held',s,current,robot.bins)
                if held_view=='wrist':self.save('held-wrist',wrist_snapshot,[],{})
                return
            if empty>=3:
                raise EmptyGrasp('Confirmed empty grasp; target remains on table')
            time.sleep(.15)
        if last_snapshot is not None:self.save('held_failed',last_snapshot,current,robot.bins)
        if last_wrist is not None:self.save('held_failed_wrist',last_wrist,[],{})
        raise Stop('Lift did not establish visible retained object and gripper contact')

    @timed('verification')
    def confirm_placed(self,part,bin,source,prior_count):
        stable = 0
        last_center = None
        deadline = time.monotonic()+12
        while time.monotonic()<deadline:
            s,remaining,bins = self.observe('placement_check',{part.kind})
            all_parts,verification_bins = self.detector.detect(s,include_bins=True)
            inside = [p for p in all_parts if p.kind==part.kind and fits_bin(p,verification_bins[part.kind])]
            absent = not any(p.kind==part.kind and np.linalg.norm(np.array(p.center)[:2]-source[:2])<.035 for p in remaining)
            center = np.mean([p.center for p in inside],axis=0) if inside else None
            unchanged = center is not None and last_center is not None and np.linalg.norm(center-last_center)<.004
            ok = absent and len(inside)==prior_count+1 and unchanged
            stable = stable+1 if ok else 0
            last_center = center
            self.log('placement_check',kind=part.kind,in_bin=len(inside),source_absent=absent,stable=stable)
            if stable>=3:
                self.bin_parts[part.kind]=inside
                return
            time.sleep(.2)
        raise Stop('Release not visually confirmed in the correct bin')

    @timed('planning')
    def position_observer(self):
        robot=self.robot
        if robot.side!=0:raise Stop('Observer view only planned for left-held C')
        measured=robot.measured();seed=measured[8:15].copy()
        pose=robot.pose(0);target=pose[:3,3]+pose[:3,:3]@robot.payload[0]
        offset,relative,_=self.wrist.calibration[1]
        relative=relative@np.diag([1,-1,-1])
        candidates=[]
        for x in (.3,.4,.5):
            for z in (.15,.25):
                camera=np.array([x,-.2,z])
                forward=target-camera;forward/=np.linalg.norm(forward)
                right=np.cross(forward,[0,0,1]);right/=np.linalg.norm(right)
                base=np.column_stack((right,np.cross(forward,right),forward))
                for roll in (0.,np.pi/4,-np.pi/4,np.pi/2,-np.pi/2,np.pi):
                    rotation=base@Rotation.from_euler('z',roll).as_matrix()@relative.T
                    point=camera-rotation@(offset-robot.kin.tool_xyz)
                    if point[1]>0 or np.linalg.norm(point-pose[:3,3])<.20:continue
                    try:
                        q=robot.solve_pose(1,point,rotation,seed)
                        for fraction in np.linspace(0,1,31):
                            robot.check_clearance(robot.kin.matrix(seed+(q-seed)*fraction,1),1)
                        candidates.append((np.linalg.norm(q-seed),q))
                    except Stop:pass
        if not candidates:raise Stop('No checked observer camera pose')
        self.observer_return=seed
        _,q=min(candidates,key=lambda x:x[0])
        q14=measured[np.r_[0:7,8:15]];q14[7:]=q
        self.log('observer_plan',q=q,target=target)
        robot.joint_path(q14,'observer_view')

    @timed('perception')
    def measure_payload(self,part,allow_observer=True):
        from .payload import screwdriver_payload
        from .synchronization import image_arm_pose
        self.robot.wait_stationary(self.robot.side)
        candidates=[]
        rejected_views={}
        for attempt in range(3):
            snapshots=[]
            try:snapshots.append(self.scene.read(candidate_stride=1))
            except (ValueError,RuntimeError) as e:self.log('payload_view_rejected',reason=str(e))
            for side in (self.robot.side,1-self.robot.side):
                try:snapshots.append(self.wrist.read(side,self.robot.table))
                except ValueError:pass
            for snapshot in snapshots:
                try:
                    tcp,joint_dt=image_arm_pose(self.scene,self.robot.kin,snapshot['stamp'],self.robot.side)
                    payload,info=screwdriver_payload(snapshot,tcp,(part.full_extent or part.extent)[0])
                    info.update(tcp=tcp,joint_dt_s=joint_dt,source=snapshot.get('source','head'))
                    candidates.append((info['shaft_points'],payload,info,snapshot))
                except ValueError as e:
                    view=snapshot.get('source','head')
                    rejected_views[view]=snapshot
                    self.log('payload_view_rejected',source=view,reason=str(e))
            if candidates:break
            time.sleep(.2)
        if not candidates:
            for view,snapshot in rejected_views.items():self.save('payload_failed_'+view,snapshot,[],{})
            if allow_observer and self.observer_return is None:
                self.position_observer()
                return self.measure_payload(part,allow_observer=False)
            raise Stop('Cannot establish in-hand full screwdriver pose')
        _,payload,info,snapshot=max(candidates,key=lambda x:x[0])
        self.robot.payload=payload
        self.robot.payload_shaft_sign=1.
        self.save('payload_pose',snapshot,[],self.robot.bins)
        self.log('payload_measured',**info,local_center=payload[0],local_rotation=payload[1],half=payload[2])
        return payload

    @timed('planning')
    def plan_release_roll(self,pose=None,initial=None,payload=None,record=True):
        robot=self.robot;side=robot.side
        if pose is None:pose=robot.pose(side)
        if payload is None:payload=robot.payload
        center=pose[:3,3]+pose[:3,:3]@payload[0]
        axis=(pose[:3,:3]@payload[1])[:,0]
        if initial is None:initial=robot.measured()[(0,8)[side]:(0,8)[side]+7]
        candidates=[]
        failures={}
        for sign in (-1,1):
            seed=initial.copy();path=[];cost=0.
            try:
                for angle in range(5,91,5):
                    turn=Rotation.from_rotvec(axis*np.deg2rad(sign*angle)).as_matrix()
                    rot=turn@pose[:3,:3];point=center+turn@(pose[:3,3]-center)
                    q=robot.solve_pose(side,point,rot,seed)
                    robot.check_clearance(robot.kin.matrix(q,side),side,payload=payload,extra_margin=.005,gripper_open=True)
                    cost+=np.linalg.norm(q-seed);seed=q
                    if angle%15==0:path.append((point,rot))
                candidates.append((cost,path))
            except Stop as e:failures[str(e)]=failures.get(str(e),0)+1
        if not candidates:
            if record:self.log('release_roll_rejected',reasons=failures)
            raise Stop('No checked motion to remove open lower finger support')
        _,path=min(candidates,key=lambda c:c[0])
        if record:self.log('release_roll_plan',axis=axis,center=center,path=[dict(point=p,rotation=r) for p,r in path])
        return pose,path

    def checked_transport_route(self,part,qhigh,q,payload,rot,goal,release_margin,reference_bias=None):
        robot=self.robot;side=robot.side
        transport_margin=np.asarray(release_margin).copy()
        elbow_reserve=part.kind=='C' and side==0 and reference_bias is not None
        if elbow_reserve:transport_margin[2]=max(transport_margin[2],C_ELBOW_IK_MARGIN)
        if reference_bias is not None and np.max(np.abs(reference_bias))>.25:
            raise Stop('Transport reference bias exceeds tracking limit')
        def check_leg(a,b):
            # C36's low-start route clipped a wall between the former 31
            # samples. Its long tip needs finer sweeps before accepting a
            # route; live actual/reference checks remain in force as well.
            samples=(max(81,int(np.ceil(np.max(np.abs(b-a))/.010))+1) if part.kind=='C' else
                     max(31,int(np.ceil(np.max(np.abs(b-a))/.025))+1))
            # Retain the fixed grid as well as the joint-step grid. C43's
            # candidate clipped a wall between the latter grid's samples.
            fractions=(np.unique(np.r_[np.linspace(0,1,samples),np.linspace(0,1,81)])
                       if part.kind=='C' else np.linspace(0,1,samples))
            for fraction in fractions:
                nominal=a*(1-fraction)+b*fraction
                # Recorded loaded transport saturated its compensation in a
                # folded, raised-shoulder posture, including after increasing
                # hard-limit reserve. Avoid that whole posture region.
                if (elbow_reserve and nominal[1]>1.2 and
                        nominal[2]-robot.kin.limits[side][2,0]<C_ELBOW_TRANSIT_MARGIN-1e-8):
                    raise Stop('Loaded left elbow lacks transport reserve')
                pose=robot.kin.matrix(nominal,side)
                robot.check_clearance(pose,side,payload=payload,extra_margin=.010)
                if part.kind=='A':
                    robot.check_clearance(pose,side,payload=payload,extra_margin=.010,gripper_open=True)
                if reference_bias is not None:
                    nominal=a*(1-fraction)+b*fraction
                    limits=robot.kin.limits[side]
                    reference=np.clip(nominal+reference_bias,limits[:,0]+.001,limits[:,1]-.001)
                    # For an occupied C bin, the starting reference may only meet the
                    # original clearance. Grow extra room as we leave it,
                    # instead of rejecting every possible departure at t=0.
                    reference_margin=(min(.002,.02*np.linalg.norm(nominal-qhigh))
                                      if part.kind=='C' and self.bin_parts.get('C') else 0.)
                    for alpha in (.5,1.):
                        robot.check_clearance(robot.kin.matrix(nominal+(reference-nominal)*alpha,side),
                                              side,payload=payload,extra_margin=reference_margin)
        def over_rim(start):
            try:
                check_leg(start,q)
                return [q]
            except Stop:
                if part.kind not in ('A','C'):raise
            start_pose=robot.kin.matrix(start,side)
            orientations=Slerp([0.,1.],Rotation.from_matrix(np.array([start_pose[:3,:3],rot])))
            for rise in (.06,.10,.14):
                for fraction in (.35,.65):
                    middle=start_pose[:3,3]*(1-fraction)+goal*fraction
                    middle[2]=max(start_pose[2,3],goal[2])+rise
                    for middle_rotation in (orientations(fraction).as_matrix(),rot):
                        try:
                            via=robot.solve_pose(side,middle,middle_rotation,start,joint_margin=transport_margin)
                            original_via=via
                            if (elbow_reserve and via[1]>1.2 and
                                    via[2]-robot.kin.limits[side][2,0]<C_ELBOW_TRANSIT_MARGIN):
                                via=via.copy();via[1]=1.15
                            check_leg(start,via);check_leg(via,q)
                            if via is not original_via:
                                self.log('transport_shoulder_corridor',original=original_via,q=via)
                            return [via,q]
                        except Stop:pass
            raise Stop('No checked transport corridor over bin wall')
        try:return over_rim(qhigh)
        except Stop:
            if part.kind!='C':raise
        # A clear low grasp can still sweep its long tip through a neighbouring
        # wall when a single intermediate pose also turns toward the release.
        # First gain clearance near the measured TCP, then use the same checked
        # over-rim search. Execute these IK configurations as joint paths so
        # Cartesian redundancy cannot drift into a saturated wrist posture.
        start_pose=robot.kin.matrix(qhigh,side)
        if start_pose[2,3]>=max(b.rim for b in robot.bins.values()):
            raise Stop('No checked transport corridor over bin wall')
        starts=[([],qhigh)]
        # The feedback compensation can put the reference close to a wall
        # even before the measured hand moves. A short vertical segment can
        # make room for the subsequent turn without changing the held pose.
        for rise in (.04,.08):
            point=start_pose[:3,3].copy();point[2]+=rise
            try:
                lifted=robot.solve_pose(side,point,start_pose[:3,:3],qhigh,joint_margin=transport_margin)
                check_leg(qhigh,lifted)
                starts.append(([lifted],lifted))
            except Stop:pass
        for prefix,start in starts:
            if prefix:
                try:
                    route=prefix+over_rim(start)
                    self.log('transport_clearance_route',vertical_prefix=True,legs=len(route))
                    return route
                except Stop:pass
            pose=robot.kin.matrix(start,side)
            orientations=Slerp([0.,1.],Rotation.from_matrix(np.array([pose[:3,:3],rot])))
            for rise in (.04,.08,.12,.16,.20):
                for fraction in (.3,.15,0.,.5):
                    point=pose[:3,3].copy();point[2]+=rise
                    try:
                        lifted=robot.solve_pose(side,point,orientations(fraction).as_matrix(),start,
                                                joint_margin=transport_margin)
                        check_leg(start,lifted)
                        route=prefix+[lifted]+over_rim(lifted)
                        self.log('transport_clearance_route',rise=rise,orientation_fraction=fraction,
                                 vertical_prefix=bool(prefix),legs=len(route))
                        return route
                    except Stop:pass
        raise Stop('No checked transport corridor over bin wall')

    @timed('planning')
    def plan_place(self,part,bin,grasp_rotation,source,high,payload_override=None,from_current=False,lift_only=False):
        robot=self.robot
        # Settled objects are obstacles throughout transport, opening and
        # retreat. A TCP XY distance alone wrongly blocks clear space above them.
        robot.settled_parts=[p for values in self.bin_parts.values() for p in values]
        side=robot.side
        start=(0,8)[side]
        source_pose=np.eye(4);source_pose[:3,:3]=grasp_rotation;source_pose[:3,3]=source
        local_center,local_rotation,half=payload=source_payload(part,source_pose,robot.table)
        if part.kind=='C':robot.payload_shaft_sign=1.
        if payload_override is not None:
            local_center,local_rotation,half=payload=payload_override
        if part.kind=='C' and abs(local_rotation[0,0])>.25:
            raise Stop('C grasp closing axis is not transverse to handle')
        if from_current:
            qhigh=robot.measured()[start:start+7].copy()
            above=robot.kin.matrix(qhigh,side)[:3,3].copy()
            robot.check_clearance(robot.kin.matrix(qhigh,side),side,payload=payload,extra_margin=.010)
            self.log('transport_start',q=qhigh,tcp=robot.kin.matrix(qhigh,side),payload=payload)
            corridor=[]
        elif part.kind in ('A','C'):
            # Both executions lift 100 mm above the table before planning
            # transport from fresh feedback. Validate that same endpoint and
            # the complete turning route below. Requiring C's obsolete fixed
            # attitude high lift rejected shorter-lever grasps in full run 36.
            above=source.copy();above[2]=robot.table+.10
            qhigh=robot.solve_pose(side,above,grasp_rotation,joint_margin=A_LIFT_JOINT_MARGIN)
            robot.check_clearance(robot.kin.matrix(qhigh,side),side,payload=payload,extra_margin=.010)
            corridor=[above]
        else:
            above,qhigh=robot.transfer_pose(side,part,source,grasp_rotation)
            corridor=[above]
        if lift_only:
            self.log('lift_corridor_plan',kind=part.kind,q=qhigh,vertices=corridor)
            return None,None,None,qhigh,above,corridor,[]
        reference_bias=robot.command[start:start+7]-qhigh if from_current and part.kind=='C' else None
        if part.kind=='C':
            from .release_optimizer import release_candidates
            cavity=getattr(self,'release_bins',{}).get('C',bin)
            # Keep the original 3mm + 2mm hard planning floor. For freshly
            # measured second rods, prefer as much landing room as the checked
            # geometry allows, up to 7mm. A rigid 7mm floor rejected C43's
            # reachable 6.62mm pose. The live release gate still requires 5mm.
            settling_margin=.003
            settling_target=(C_OCCUPIED_SETTLING_MARGIN+C_RELEASE_RESERVE+.00005
                             if self.bin_parts.get('C') and payload_override is not None else None)
            # With an occupied bin there is less room to reorient after a
            # small in-hand pivot. Reserve clearance before closing, instead
            # of relying on a source pose that barely clears the open palm.
            source_drop_reserve=.003 if self.bin_parts.get('C') else .002
            for drop_reserve in ((.002,.0003) if payload_override is not None else (source_drop_reserve,)):
                candidates=[];rejected={}
                proposals=sorted(release_candidates(robot.kin,side,qhigh,getattr(self,'release_seeds',[]),payload,
                        part.handle_length,cavity,bin.rim,robot.table,RELEASE_JOINT_MARGIN,C_RELEASE_MARGIN,
                        C_RELEASE_RESERVE,self.bin_parts.get('C',[]),drop_reserve=drop_reserve,
                        reference_bias=reference_bias,settling_margin=settling_margin,
                        settling_target=settling_target),key=lambda item:item[0])
                # Routes cannot improve the release score. Check candidates
                # in score order and stop at the first feasible route, which
                # selects the same minimum without planning all other routes.
                for score,q in proposals:
                    pose=robot.kin.matrix(q,side);rot=pose[:3,:3];goal=pose[:3,3]
                    try:
                        robot.check_clearance(pose,side,payload=payload,extra_margin=.010)
                        robot.check_clearance(pose,side,payload=payload,extra_margin=.010,gripper_open=True)
                        route=self.checked_transport_route(part,qhigh,q,payload,rot,goal,RELEASE_JOINT_MARGIN,reference_bias)
                    except Stop as ex:
                        rejected[str(ex)]=rejected.get(str(ex),0)+1
                        continue
                    candidates.append((score,q,pose,route))
                    break
                if candidates:
                    _,q,pose,route=min(candidates,key=lambda item:item[0])
                    if self.bin_parts.get('C') and payload_override is not None:
                        from .release_refinement import refine_held_clearance
                        improvement=refine_held_clearance(robot.kin,side,qhigh,q,payload,part.handle_length,
                            cavity,bin.rim,robot.table,RELEASE_JOINT_MARGIN,reference_bias,drop_reserve)
                        if improvement is not None:
                            refined_q,margins=improvement
                            refined_pose=robot.kin.matrix(refined_q,side)
                            try:
                                robot.check_clearance(refined_pose,side,payload=payload,extra_margin=.010)
                                robot.check_clearance(refined_pose,side,payload=payload,extra_margin=.010,gripper_open=True)
                                refined_route=self.checked_transport_route(part,qhigh,refined_q,payload,
                                    refined_pose[:3,:3],refined_pose[:3,3],RELEASE_JOINT_MARGIN,reference_bias)
                            except Stop as ex:
                                self.log('held_release_margin_retained',reason=str(ex))
                            else:
                                q,pose,route=refined_q,refined_pose,refined_route
                                self.log('held_release_margin_improved',**margins)
                    rot=pose[:3,:3];goal=pose[:3,3]
                    self.release_seeds=(getattr(self,'release_seeds',[])+[q.copy()])[-8:]
                    self.log('continuous_release_plan',candidates=len(candidates),proposals=len(proposals),
                             rejected=rejected,drop_reserve=drop_reserve,settling_margin=settling_margin,
                             settling_target=settling_target)
                    self.log('place_plan',rotation=rot,goal=goal,q=q,high_q=qhigh,transport_qs=route,
                             local_center=local_center,local_rotation=local_rotation,half=half)
                    return rot,goal,q,qhigh,above,corridor,route
                self.log('continuous_release_rejected',reasons=rejected,drop_reserve=drop_reserve)
            if payload_override is None:
                raise Stop('No source-profile checked release from this grasp')
        candidates=[]
        failures={}
        count=self.initial_counts[part.kind]+sum(p['kind']==part.kind for p in self.completed)
        cavity=getattr(self,'release_bins',{}).get(part.kind,bin)
        # A stationary, side-opening hand needs no empirical deflection from
        # the old lower-finger roll. Landing is still verified with fresh RGB-D.
        bias=None
        if part.kind=='C':
            # Keep the pad separation nearly horizontal during opening. The
            # measured object axis and gravity sweep are checked separately.
            tool_x=local_rotation.T[:,0]
            roll0=np.arctan2(tool_x[1],tool_x[2])
            rolls=([roll0+np.deg2rad(delta) for delta in (60,75,90,45,-60,-75,-90,-45,120,-120,135,-135)]
                   if payload_override is not None else (roll0,roll0+np.pi))
            rotations=[(0.,Rotation.from_euler('z',np.deg2rad(angle)).as_matrix()@
                        Rotation.from_euler('y',pitch).as_matrix()@
                        Rotation.from_euler('x',roll).as_matrix()@local_rotation.T)
                       for pitch in ((.26,-.26,.34,-.34,.40,-.40,.46,-.46,.52,-.52,.58,-.58,.65,-.65)
                                     if payload_override is not None else (.26,-.26)) for roll in rolls
                       for base in (40,120,220,300) for angle in range(base,base+21,1 if payload_override is not None else 2)]
            rotations=[(penalty+gap*20,rot,gap) for gap in
                       ((.045,.055,.065,.085,.030,.020) if payload_override is not None else (.065,.085))
                       for penalty,rot in rotations]
            if self.bin_parts.get('C'):
                occupied_axes=[np.asarray(p.axis) for p in self.bin_parts['C']]
                def parallelism(item):
                    axis=(item[1]@local_rotation)[:2,0]
                    axis=axis/max(np.linalg.norm(axis),1e-9)
                    return max(abs(np.dot(axis,a)) for a in occupied_axes)
                def handle_separation(item):
                    shift=(2*half[0]-min(part.handle_length,2*half[0]))/2
                    handle=np.asarray(bin.center)-(item[1]@local_rotation)[:2,0]*shift
                    return min(np.linalg.norm(handle-np.asarray(p.center)[:2]) for p in self.bin_parts['C'])
                # Prefer the other diagonal so handles remain separated after
                # falling, rather than repeatedly dropping onto the same rod.
                rotations.sort(key=lambda item:(round(parallelism(item),6),-handle_separation(item),item[0]))
        else:
            release_yaws=np.arange(0.,2*np.pi,np.pi/12) if part.kind=='B' else (0.,np.pi/2,np.pi,-np.pi/2)
            rotations=[(abs(tilt)*.2,Rotation.from_euler('y',tilt).as_matrix()@Rotation.from_euler('z',yaw).as_matrix(),.03)
                       for tilt in (-.52,-.78,-1.05,-1.3,-1.57) for yaw in release_yaws]
        rotations=[(penalty,rot,gap,False) for penalty,rot,gap in rotations]
        if part.kind=='A' and self.bin_parts.get('A'):
            # A settled upright part can occupy the near edge. Search the
            # opposite edge only if all preferred routes are unavailable.
            rotations += [(penalty,rot,gap,True) for penalty,rot,gap,_ in rotations]
        for penalty,rot,gap,far_edge in rotations:
            if far_edge and candidates:break
            if part.kind=='C' and abs(rot[2,0])>C_OPENING_PLANNING_LIMIT:continue
            extent=np.abs(rot@local_rotation)@half
            center=np.r_[bin.center,bin.rim+extent[2]+gap]
            margin=(C_RELEASE_MARGIN if payload_override is not None else .003) if part.kind=='C' else .008
            held_offsets=None
            if part.kind=='C' and payload_override is not None:
                axis=(rot@local_rotation)[:,0]
                held_offsets=(c_profile(half,part.handle_length)@(rot@local_rotation).T)[:,:2]
                try:center[:2]=release_center(cavity,cavity,axis,half,extent,margin,part.handle_length,rot@local_rotation,bias,
                                             reserve=C_RELEASE_RESERVE,settling_margin=settling_margin)
                except ValueError:continue
                if not settling_fits(cavity,center,axis,half,margin=settling_margin,handle_length=part.handle_length,bias=bias):continue
            if part.kind!='C':
                center[0]=(bin.upper[0]-extent[0]-.016 if far_edge else bin.lower[0]+extent[0]+.016)
                # A46's tilted bracket landed against the lateral wall even
                # though its held envelope had 12mm clearance. Leave room for
                # that settling motion; the two slots and occupied-part checks
                # still keep the brackets separated.
                slot_offset=.03 if part.kind=='A' else .05
                side_reserve=.025 if part.kind=='A' else .012
                center[1]+=(-slot_offset if count==0 else slot_offset)
                lower_y=bin.lower[1]+extent[1]+side_reserve
                upper_y=bin.upper[1]-extent[1]-side_reserve
                if lower_y>upper_y:continue
                center[1]=np.clip(center[1],lower_y,upper_y)
                occupied=False
                for settled in self.bin_parts.get(part.kind,[]):
                    settled_axis=np.array(settled.axis)
                    settled_basis=np.column_stack((settled_axis,[-settled_axis[1],settled_axis[0]]))
                    settled_half=np.abs(settled_basis)@np.array(settled.full_extent or settled.extent)/2
                    if np.all(np.abs(center[:2]-np.array(settled.full_center or settled.center)[:2])<extent[:2]+settled_half+.005):
                        occupied=True;break
                if occupied:continue
            if not held_fits(cavity if part.kind=='C' and payload_override is not None else bin,center,extent,margin,held_offsets):
                continue
            goal=center-rot@local_center
            if part.kind!='C':goal[2]=max(goal[2],bin.rim+.07)
            try:
                if part.kind=='C':
                    nominal=np.eye(4);nominal[:3,:3]=rot;nominal[:3,3]=goal
                    if payload_override is not None and not free_drop_clear(nominal,payload,part.handle_length,robot.table):
                        continue
                    robot.check_clearance(nominal,side,payload=payload,extra_margin=.010,gripper_open=True)
                # Loaded wrists need reference headroom to compensate MPC bias.
                release_margin=RELEASE_JOINT_MARGIN if part.kind in ('A','C') else .003
                q=robot.solve_pose(side,goal,rot,qhigh,joint_margin=release_margin,reference_bias=reference_bias)
                if part.kind=='B' and min(q[6]-robot.kin.limits[side][6,0],robot.kin.limits[side][6,1]-q[6])<.08:
                    raise Stop('B release wrist lacks tracking reserve')
                robot.check_clearance(robot.kin.matrix(q,side),side,payload=payload,extra_margin=.010)
                if part.kind=='C':
                    if not side_opening_clear(robot.kin.matrix(q,side)):
                        raise Stop('C opening axis is too steep')
                    robot.check_clearance(robot.kin.matrix(q,side),side,payload=payload,extra_margin=.010,gripper_open=True)
                    if payload_override is not None:
                        if not free_drop_clear(robot.kin.matrix(q,side),payload,part.handle_length,robot.table):
                            raise Stop('Open fingers obstruct the gravity release')
                if part.kind in ('A','B') and candidates:
                    # Every detoured route is at least this direct distance.
                    # Skip only candidates which cannot beat the best checked
                    # score; preserve possible ties and the selected motion.
                    bound=np.linalg.norm(q-qhigh)+penalty
                    best=min(item[0] for item in candidates)
                    if bound>best+1e-9:
                        self.log('release_route_pruned',kind=part.kind,lower_bound=bound,best_score=best)
                        continue
                route=self.checked_transport_route(part,qhigh,q,payload,rot,goal,release_margin,reference_bias)
                score=sum(np.linalg.norm(b-a) for a,b in zip([qhigh]+route[:-1],route))+penalty
                if part.kind=='C':
                    clearance=min(np.min(center[:2]-extent[:2]-bin.lower),np.min(np.array(bin.upper)-center[:2]-extent[:2]))
                    score-=100*clearance
                    if self.bin_parts.get('C'):score-=20*min(handle_separation((penalty,rot,gap)),.10)
                candidates.append((score,rot,goal,q,route))
                if part.kind=='C' and len(candidates)>=4:break
            except Stop as e:
                reason=str(e);failures[reason]=failures.get(reason,0)+1
        if not candidates:
            self.log('place_rejected',kind=part.kind,reasons=failures)
            raise Stop('No reachable collision-checked release pose fits the visible bin')
        _,rot,goal,q,route=min(candidates,key=lambda item:item[0])
        if part.kind=='C':self.release_seeds=(getattr(self,'release_seeds',[])+[q.copy()])[-8:]
        self.log('place_plan',rotation=rot,goal=goal,q=q,high_q=qhigh,transport_qs=route,local_center=local_center,local_rotation=local_rotation,half=half)
        return rot,goal,q,qhigh,above,corridor,route

    def select_arm(self,part,bin):
        robot=self.robot
        source_side=1 if part.center[1]<0 else 0
        preferred={'A':1,'C':0}.get(part.kind,source_side)
        robot.target,robot.side=part,source_side
        if source_side==preferred:return source_side
        # Test both arms before deciding that a cross-side object needs a
        # transfer. Planning is read-only; no hover or gripper command is sent.
        hover=np.asarray(part.center).copy()
        hover[2]=robot.table+({'A':.12,'B':.18,'C':.10}[part.kind])
        high=max(b.rim for b in robot.bins.values())+(.055 if part.kind=='A' else .11)
        for side in (source_side,preferred):
            robot.side=side
            try:
                robot.orient_for_grasp(side,part,hover,execute=False,
                    validator=lambda rot,source:self.plan_place(part,bin,rot,source,high))
                self.log('direct_route_checked',kind=part.kind,side=side,cross_side=True,reachable=True)
                return side
            except Stop as ex:
                self.log('direct_route_checked',kind=part.kind,side=side,cross_side=True,reachable=False,reason=str(ex))
        self.log('handover_event',state='route_required',donor=source_side,receiver=preferred,
                 reason='Both direct routes were rejected',release_authorized=False)
        raise Stop('Cross-side target needs a collision-checked handover rendezvous')

    def pick_place(self,part,bin,snapshot):
        robot = self.robot
        side = self.select_arm(part,bin)
        robot.target,robot.side = part,side
        reference_bins=dict(robot.bins)
        source = np.array(part.center)
        axis = np.array(part.axis)
        # Local X closes the fingers; local Y follows the object long axis.
        rotation = np.array([[axis[1],axis[0],0],[-axis[0],axis[1],0],[0,0,1]])
        # Equivalent opposing jaw orientation: choose the nearer wrist heading.
        current = robot.pose(side)
        choices = [rotation,rotation@np.diag([-1,-1,1])]
        hover = source.copy()
        hover[2] = robot.table+({'A':.12,'B':.18,'C':.10}[part.kind])
        self.log('select',part=asdict(part),side=side,bin=asdict(bin))
        rotation,grasp_point = (robot.observation_hover(side,part,hover) if part.kind=='A' else
                                robot.orient_for_grasp(side,part,hover))
        hover[:2]=grasp_point[:2]
        robot.cartesian(side,hover,rotation,'hover')
        # Relocalize at hover. Occlusion of an irregular mesh can shift its
        # visible centroid; accept only one nearby instance and replan below.
        s,parts,bins = self.observe('pregrasp')
        shifts=np.array([np.array(bins[k].center)-reference_bins[k].center for k in reference_bins if k in bins])
        shift=np.median(shifts,axis=0)
        support=np.linalg.norm(shifts-shift,axis=1)<.012
        if support.sum()<2 or np.linalg.norm(shift)>.03:
            raise Stop('No consistent visual reference for robot/table motion')
        source[:2]+=shift
        self.log('table_reference_shift',shift=shift,support=int(support.sum()))
        matches = [p for p in parts if p.kind==part.kind and np.linalg.norm(np.array(p.center)-source)<.030]
        incomplete=(len(matches)==1 and part.kind in ('A','C') and
                    (matches[0].full_extent or matches[0].extent)[0]<(part.full_extent or part.extent)[0]*.95)
        if part.kind in ('A','C') and len(matches)<=1:
            partial=self.partial_target(part,source,snapshot['table_z_base_m'])
            if partial is not None:
                for old_match in matches:
                    self.robot.parts=[p for p in self.robot.parts if p is not old_match]
                matches=[partial]
            elif incomplete:
                matches=[]
        if len(matches)!=1:
            raise Stop('Target moved or became ambiguous before descent')
        part = matches[0]
        axis=np.array(part.axis)
        self.log('target_refined',displacement=np.array(part.center)-source,part=asdict(part))
        robot.target = part
        source = np.array(part.center)
        bin=bins[part.kind]
        # Wrist refinement changes both the centre and the principal axis.
        # Revalidate the grasp against that geometry before descending.
        hover=source.copy();hover[2]=robot.table+({'A':.12,'B':.18,'C':.10}[part.kind])
        high=max(b.rim for b in bins.values())+(.055 if part.kind=='A' else .11)
        routes={}
        def validate_route(candidate_rotation,candidate_source):
            routes['selected']=self.plan_place(part,bin,candidate_rotation,candidate_source,high)
        rotation,source=robot.orient_for_grasp(side,part,hover,validator=validate_route,
                                             height_adjust=0.)
        place_rotation,place_goal,place_q,return_q,high_point,corridor,transport_qs=(routes['selected'] if routes else
                                                            self.plan_place(part,bin,rotation,source,high))
        current=robot.pose(side)[:3,3]
        aligned=source.copy();aligned[2]=current[2]
        robot.cartesian(side,aligned,rotation,'align_above')
        for height in np.linspace(aligned[2],source[2],max(2,int(np.ceil(abs(aligned[2]-source[2])/.01))+1))[1:]:
            waypoint=source.copy();waypoint[2]=height
            robot.cartesian(side,waypoint,rotation,'descend',settle_frames=1)
        if part.kind=='A':
            source,rotation=robot.finish_grasp_descent(side,source,rotation)
        else:
            robot.cartesian(side,source,rotation,'descend',position_tolerance=.0025 if part.kind=='C' else .002)
        robot.gripper(side,True,'close',contact_stop=part.kind=='C')
        grasp=robot.pose(side)
        robot.payload=source_payload(part,grasp,robot.table)
        if part.kind=='C':robot.payload_shaft_sign=1.
        robot.held = True  # collision context only, not success evidence
        lift=self.lift_grasp(part,side,source,rotation,grasp)
        self.confirm_held(part,side,source)
        if part.kind=='A':
            place_rotation,place_goal,place_q,return_q,high_point,corridor,transport_qs=self.plan_place(
                part,bin,rotation,source,high,payload_override=robot.payload,from_current=True)
        if part.kind=='C':
            # Limit squeezing while supported by the table, then establish
            # transport grip only after visual lift confirmation.
            robot.gripper(side,True,'secure_grasp')
            self.confirm_held(part,side,source)
            payload=self.measure_payload(part)
            # C12 reached a different redundant arm posture from the planned
            # fixed-attitude high lift and saturated both wrist references.
            # After the clear vertical lift, finish_placement plans a checked
            # joint corridor from actual feedback and the fresh payload. It
            # may turn and rise together while preserving the full envelope.
            high_point=robot.pose(side)[:3,3].copy()
            corridor=[]
            self.log('transport_from_clear_lift',kind=part.kind,pose=robot.pose(side),payload=payload)
        # Transfer high enough for the full finger and carried object envelope.
        low=lift.copy()
        for vertex_index,vertex in enumerate(corridor):
            fractions=np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(vertex-low)/.02))+1))[1:]
            for fraction in fractions:
                frames=5 if vertex_index==len(corridor)-1 and fraction==1 else 1
                robot.cartesian(side,low+(vertex-low)*fraction,rotation,'clear_rims',settle_frames=frames)
            low=vertex.copy()
        return self.finish_placement(part,bin,source,rotation,high,place_rotation,place_goal,place_q,return_q,high_point,transport_qs)

    def lift_grasp(self,part,side,source,rotation,grasp):
        robot=self.robot
        lift = source.copy()
        lift[2] = robot.table+(.15 if part.kind=='B' else .10)
        lift_rotation=rotation
        if part.kind=='A':
            # Closing around an irregular mesh can change the measured wrist
            # pose. First lift vertically from that pose; correcting the old
            # orientation at table height can sweep the held part downward.
            lift[:2]=grasp[:2,3]
            lift_rotation=grasp[:3,:3]
        # Bound the Cartesian lift horizon as on descent: a large vertical
        # error can produce a lateral transient near adjacent table objects.
        current_height=robot.pose(side)[2,3]
        for height in np.linspace(current_height,lift[2],max(2,int(np.ceil(abs(lift[2]-current_height)/.02))+1))[1:]:
            waypoint=lift.copy();waypoint[2]=height
            robot.cartesian(side,waypoint,lift_rotation,'lift',settle_frames=5 if height==lift[2] else 1)
        if part.kind=='A':
            lift[:2]=source[:2]
            actual=robot.pose(side)
            orientations=Slerp([0.,1.],Rotation.from_matrix(np.array([actual[:3,:3],rotation])))
            robot.stage='lift_align'
            # Check the entire alignment sweep above the table before rotating.
            # The table-contact allowance used only for initial lift is over.
            for fraction in np.linspace(0,1,21):
                pose=np.eye(4);pose[:3,:3]=orientations(fraction).as_matrix()
                pose[:3,3]=actual[:3,3]*(1-fraction)+lift*fraction
                robot.check_clearance(pose,side,payload=robot.payload,table_reserve=.003)
            self.log('lift_alignment_checked',kind=part.kind,measured_grasp=grasp,clear_pose=actual)
            robot.cartesian(side,lift,rotation,'lift_align')
        return lift

    def finish_placement(self,part,bin,source,rotation,high,place_rotation,place_goal,place_q,return_q,high_point,transport_qs):
        robot=self.robot;side=robot.side
        lift=high_point.copy()
        count = self.initial_counts[part.kind]+sum(x['kind']==part.kind for x in self.completed)
        if part.kind=='C':
            payload=self.measure_payload(part)
            for clearance_lift in range(4):
                pose=robot.pose(side)
                try:
                    robot.check_transport_start(robot.measured(),payload)
                    break
                except Stop:
                    if clearance_lift==3:raise
                    goal,clear_rotation=robot.transport_clearance_goal(robot.measured(),payload)
                    self.log('raise_transport_clearance',goal=goal)
                    robot.cartesian(side,goal,clear_rotation,'raise_transport_clearance',position_tolerance=.003)
                    payload=self.measure_payload(part)
            for _ in range(3):
                measured=robot.measured();start=(0,8)[side];arm=measured[start:start+7].copy()
                if not (side==0 and arm[2]-robot.kin.limits[side][2,0]<.01 and arm[1]>1.2):break
                unfolded=arm.copy();unfolded[1]-=.15
                for fraction in np.linspace(0,1,31):
                    robot.check_clearance(robot.kin.matrix(arm+(unfolded-arm)*fraction,side),side,payload=payload,extra_margin=.010)
                q14=measured[np.r_[0:7,8:15]];q14[:7]=unfolded
                self.log('unfold_high_arm',q=unfolded)
                robot.joint_path(q14,'unfold_high_arm')
                payload=self.measure_payload(part)
            place_rotation,place_goal,place_q,return_q,high_point,_,transport_qs=self.plan_place(
                part,bin,rotation,source,high,payload_override=payload,from_current=True)
            lift=high_point.copy()
        q14=robot.measured()[np.r_[0:7,8:15]]
        q14[side*7:side*7+7]=return_q
        if part.kind!='C':robot.joints(q14,'align_transport_start',tolerance=.025,pose_tolerance=.006)
        return_path=[return_q]+transport_qs[:-1]
        for waypoint in transport_qs:
            q14[side*7:side*7+7]=waypoint
            robot.joint_path(q14,'transport_configuration',settle=part.kind!='C',pose_tracking=part.kind=='C')
        robot.cartesian(side,place_goal,place_rotation,'release_align',
                        position_tolerance=.0015 if part.kind=='C' else .007,
                        orientation_tolerance=.005 if part.kind=='C' else .10)
        if part.kind=='C':
            for _ in range(10):robot.emit(robot.command.copy())
            # C39 passed production with a 4.97mm predicted landing margin,
            # but the second rod's observed footprint later had only 2.84mm.
            # Keep extra landing clearance when placing onto an occupied bin.
            # The independent verifier retains its original 3mm requirement.
            settling_margin=C_OCCUPIED_SETTLING_MARGIN if count else .003
            corrections=stable=0
            last_center=last_axis=None
            for attempt in range(12):
                roll_blocked=False
                payload=self.measure_payload(part)
                pose=robot.pose(side)
                actual_center=pose[:3,3]+pose[:3,:3]@payload[0]
                actual_half=np.abs(pose[:3,:3]@payload[1])@payload[2]
                axis_vertical=abs((pose[:3,:3]@payload[1])[2,0])
                actual_axis=(pose[:3,:3]@payload[1])[:,0]
                held_offsets=(c_profile(payload[2],part.handle_length)@(pose[:3,:3]@payload[1]).T)[:,:2]
                cavity=getattr(self,'release_bins',{}).get('C',bin)
                bias=None
                settles_inside=settling_fits(cavity,actual_center,actual_axis,payload[2],margin=settling_margin,handle_length=part.handle_length,bias=bias)
                drop_clear=free_drop_clear(pose,payload,part.handle_length,robot.table)
                opening_clear=side_opening_clear(pose)
                held_inside=held_fits(cavity,actual_center,actual_half,C_RELEASE_MARGIN,held_offsets)
                if (held_inside and axis_vertical<.8 and settles_inside and drop_clear and opening_clear):
                    unchanged=(last_center is not None and np.linalg.norm(actual_center-last_center)<.003
                               and np.dot(actual_axis,last_axis)>np.cos(np.deg2rad(3)))
                    stable=stable+1 if unchanged else 1
                    last_center,last_axis=actual_center,actual_axis
                    self.log('release_payload_verified',center=actual_center,half=actual_half,axis_vertical=axis_vertical,stable=stable,
                             predicted_settled_corners=settled_corners(actual_center,actual_axis,payload[2],part.handle_length,bias),support_bias=bias,
                             free_drop_clear=drop_clear,opening_vertical=float(pose[2,0]),settling_margin=settling_margin)
                    if stable>=3:
                        release_pose,release_roll_path=pose,[]
                        self.log('gravity_release_plan',center=actual_center,axis=actual_axis,opening_vertical=float(pose[2,0]))
                        break
                    else:
                        time.sleep(.3)
                        continue
                stable=0;last_center=last_axis=None
                self.log('release_payload_rejected',center=actual_center,axis=actual_axis,held_inside=held_inside,
                         settles_inside=settles_inside,free_drop_clear=drop_clear,opening_clear=opening_clear,
                         opening_vertical=float(pose[2,0]),corrections=corrections,
                         cavity=asdict(cavity),settling_margin=settling_margin)
                if corrections>=3:raise Stop('No verified release and retreat after three corrections')
                previous_q=robot.measured()[np.r_[0:7,8:15]][side*7:side*7+7].copy()
                if not roll_blocked and drop_clear and axis_vertical<.8 and opening_clear:
                    # Correct translation without rotating a round handle that
                    # can pivot between the pads and invalidate a new rotation.
                    place_rotation=pose[:3,:3]
                    place_goal=pose[:3,3].copy()
                    try:compensated=release_center(cavity,cavity,actual_axis,payload[2],actual_half,C_RELEASE_MARGIN,
                                                   part.handle_length,pose[:3,:3]@payload[1],bias,reserve=C_RELEASE_RESERVE,
                                                   settling_margin=settling_margin)
                    except ValueError:compensated=None
                    if compensated is not None and not settling_fits(cavity,np.r_[compensated,actual_center[2]],actual_axis,payload[2],margin=settling_margin,handle_length=part.handle_length,bias=bias):
                        # Translation alone cannot make this orientation safe.
                        compensated=None
                    if compensated is None:
                        place_rotation,place_goal,place_q,_,_,_,transport_qs=self.plan_place(
                            part,bin,rotation,source,high,payload_override=payload,from_current=True)
                    else:
                        place_goal[:2]+=compensated-actual_center[:2]
                        reference_bias=robot.command[(0,8)[side]:(0,8)[side]+7]-previous_q
                        try:
                            place_q=robot.solve_pose(side,place_goal,place_rotation,previous_q,
                                                    joint_margin=RELEASE_JOINT_MARGIN,reference_bias=reference_bias)
                            transport_qs=self.checked_transport_route(part,previous_q,place_q,payload,
                                place_rotation,place_goal,RELEASE_JOINT_MARGIN,reference_bias)
                            self.log('release_translation_correction',goal=place_goal,center=actual_center)
                        except Stop as ex:
                            self.log('release_translation_rejected',reason=str(ex))
                            place_rotation,place_goal,place_q,_,_,_,transport_qs=self.plan_place(
                                part,bin,rotation,source,high,payload_override=payload,from_current=True)
                else:
                    place_rotation,place_goal,place_q,_,_,_,transport_qs=self.plan_place(
                        part,bin,rotation,source,high,payload_override=payload,from_current=True)
                q14=robot.measured()[np.r_[0:7,8:15]]
                return_path.extend([previous_q]+transport_qs[:-1])
                for waypoint in transport_qs:
                    q14[side*7:side*7+7]=waypoint
                    robot.joint_path(q14,'payload_release_correction',settle=False,pose_tracking=True)
                robot.cartesian(side,place_goal,place_rotation,'release_align',position_tolerance=.0015,
                                orientation_tolerance=.005)
                corrections+=1
                for _ in range(10):robot.emit(robot.command.copy())
            else:raise Stop('Measured full payload does not fit bin before release')
        robot.retained=False
        robot.gripper(side,False,'release',release_step=1. if part.kind=='C' else .04)
        if part.kind=='C':
            for point,rot in release_roll_path:
                robot.cartesian(side,point,rot,'remove_finger_support',position_tolerance=.003)
        robot.held = False
        robot.payload=None
        robot.target = None
        if part.kind=='C' and release_roll_path:
            for point,rot in reversed(release_roll_path[:-1]):
                robot.cartesian(side,point,rot,'release_roll_return',position_tolerance=.004)
            robot.cartesian(side,release_pose[:3,3],release_pose[:3,:3],'release_roll_return')
        park = lift.copy()
        q14=robot.measured()[np.r_[0:7,8:15]]
        pending=list(reversed(return_path))
        while pending:
            current=robot.measured()[np.r_[0:7,8:15]][side*7:side*7+7]
            selected=None
            for index in range(len(pending)-1,-1,-1):
                try:
                    sample_count=max(31,int(np.ceil(np.max(np.abs(pending[index]-current))/.02))+1)
                    for fraction in np.linspace(0,1,sample_count):
                        robot.check_clearance(robot.kin.matrix(current+(pending[index]-current)*fraction,side),side,extra_margin=.005)
                    selected=index;break
                except Stop:continue
            if selected is None:
                if part.kind!='C':raise Stop('No checked empty-hand return segment')
                # C38's open fingers clipped a wall midway along the joint
                # return although both endpoint poses were clear. Try the
                # completely checked pose corridor to the original lift pose.
                waypoints=robot.plan_empty_cartesian_return(side,pending[-1])
                for index,(point,rot) in enumerate(waypoints):
                    robot.cartesian(side,point,rot,'retreat_cartesian',position_tolerance=.003,
                                    orientation_tolerance=.02,settle_frames=5 if index==len(waypoints)-1 else 1)
                pending=[]
                break
            waypoint=pending[selected];pending=pending[selected+1:]
            self.log('empty_return_waypoint',skipped=selected,q=waypoint)
            q14[side*7:side*7+7]=waypoint
            robot.joint_path(q14,'retreat_configuration')
        retreat_rotation=robot.kin.matrix(return_q,side)[:3,:3] if part.kind=='C' else rotation
        robot.cartesian(side,park,retreat_rotation,'retreat_view')
        q14=robot.measured()[np.r_[0:7,8:15]]
        q14[side*7:side*7+7]=np.deg2rad([30,10 if side==0 else -10,10 if side==0 else -10,-120,-60 if side==0 else 60,0,0])
        robot.joints(q14,'park_for_observation')
        if self.observer_return is not None:
            q14=robot.measured()[np.r_[0:7,8:15]]
            q14[7:]=self.observer_return
            robot.joint_path(q14,'observer_return')
            self.observer_return=None
        self.confirm_placed(part,bin,source,count)
        self.completed.append({'kind':part.kind,'source':source.tolist(),'bin':bin.kind})
        self.log('completed',count=len(self.completed),ledger=self.completed)

    def resume_held(self,checkpoint):
        import json
        from pathlib import Path
        from .perception import Part
        rows=[json.loads(line) for line in Path(checkpoint).read_text().splitlines()]
        commands=[e for e in rows if e['event']=='command']
        if not commands or commands[-1].get('stage') not in ('transport_configuration','unfold_high_arm','clear_rims','raise_transport_clearance','release_align','payload_release_correction','observer_view','secure_grasp'):
            raise Stop('Only an interrupted high transport can be resumed')
        part=Part(**[e['part'] for e in rows if e['event']=='target_refined'][-1])
        if part.kind!='C':raise Stop('Held continuation only supports C')
        previous=[e for e in rows if e['event']=='observation'][-1]['bins']
        grasp=[e for e in rows if e['event']=='grasp_plan'][-1]
        from .bin_geometry import cavity_bounds
        labels=('pregrasp','select','select_after_scan','placement_check','initial_occupancy')
        for file in sorted(Path(checkpoint).parent.glob('*.npz'),reverse=True):
            if not any(file.name.endswith('-'+label+'.npz') for label in labels):continue
            prior=dict(np.load(file));_,prior_bins=self.detector.detect(prior)
            if not set('ABC').issubset(prior_bins):continue
            cavity=cavity_bounds(prior,masks(prior['rgb'])[1]['C'],prior_bins['C'])
            if cavity.cavity is not None:
                self.release_cavity_reference=(cavity,prior_bins)
                self.log('resume_cavity_reference',file=str(file),stamp=cavity.stamp)
                break
        snapshot,_,bins=self.observe('resume_geometry')
        shifts=np.array([np.array(bins[k].center)-previous[k]['center'] for k in ('A','B')])
        shift=shifts.mean(0)
        if np.linalg.norm(shifts[0]-shifts[1])>.01 or np.linalg.norm(shift)>.03:
            raise Stop('Held continuation scene registration inconsistent')
        all_parts,verification_bins=self.detector.detect(snapshot,True)
        self.initial_counts={k:sum(p.kind==k and fits_bin(p,verification_bins[k]) for p in all_parts) for k in 'ABC'}
        expected,has_completed=checkpoint_counts(rows)
        self.bin_parts={k:[p for p in all_parts if p.kind==k and fits_bin(p,verification_bins[k])] for k in 'ABC'}
        if self.initial_counts['C']>expected['C']:
            if self.initial_counts['C']>=2 or any(self.initial_counts[k]!=expected[k] for k in ('A','B')):
                raise Stop('Unexpected occupancy before held continuation')
            for _ in range(2):
                time.sleep(.3);fresh,_,_=self.observe('resume_occupancy_confirmation')
                observed,observed_bins=self.detector.detect(fresh,True)
                counts={k:sum(p.kind==k and fits_bin(p,observed_bins[k]) for p in observed) for k in 'ABC'}
                if counts!=self.initial_counts:raise Stop('Held continuation occupancy is not stable')
            self.log('resume_refreshed_occupancy',recorded=expected,verified=self.initial_counts)
            expected=self.initial_counts.copy()
        if self.initial_counts!=expected:
            if (any(self.initial_counts[k]!=expected[k] for k in ('A','B')) or
                    self.initial_counts['C']>expected['C']):
                raise Stop('Bin occupancy changed before held continuation')
            evidence_label='placement_check' if has_completed else 'initial_occupancy'
            prior_files=sorted(Path(checkpoint).parent.glob('*'+evidence_label+'.npz'))
            if not prior_files:raise Stop('Checkpoint lacks prior occupancy RGB-D evidence')
            prior_file=prior_files[-1]
            prior=dict(np.load(prior_file));prior_parts,prior_bins=self.detector.detect(prior,True)
            known=[p for p in prior_parts if p.kind=='C' and fits_bin(p,prior_bins['C'])]
            if len(known)!=expected['C']:raise Stop('Prior C occupancy is not supported by recorded RGB-D')
            a=np.array([prior_bins[k].center for k in ('A','B')]);b=np.array([bins[k].center for k in ('A','B')])
            u,_,vt=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)));registration=vt.T@u.T
            if np.linalg.det(registration)<0:vt[-1]*=-1;registration=vt.T@u.T
            translation=b.mean(0)-registration@a.mean(0)
            if abs(np.arctan2(registration[1,0],registration[0,0]))>.05 or np.linalg.norm(translation)>.05:
                raise Stop('Prior occupied geometry registration changed too much')
            dz=float(snapshot['table_z_base_m']-prior['table_z_base_m'])
            def transform(point):return np.r_[registration@np.asarray(point)[:2]+translation,point[2]+dz].tolist()
            self.bin_parts['C']=[replace(p,center=transform(p.center),full_center=transform(p.full_center),
                axis=(registration@p.axis).tolist(),top=p.top+dz,
                footprint=(np.asarray(p.footprint)@registration.T+translation).tolist()) for p in known]
            self.log('resume_occluded_prior_occupancy',visible=self.initial_counts,recorded=expected)
            self.initial_counts=expected.copy()
        robot=self.robot;robot.side=0;robot.target=part
        recorded_payload=next((e for e in reversed(rows) if e['event']=='payload_measured'),None)
        if recorded_payload is not None:
            robot.payload=tuple(np.asarray(recorded_payload[k],dtype=float) for k in ('local_center','local_rotation','half'))
            if not all(np.isfinite(value).all() for value in robot.payload):raise Stop('Invalid checkpoint payload geometry')
        robot.payload_shaft_sign=1.
        if not .05<robot.measured()[7]<.95 or robot.measured()[15]>.02:
            raise Stop('Held continuation gripper state inconsistent')
        if robot.pose(0)[2,3]<robot.table+.08:raise Stop('Held continuation requires a lifted hand')
        source=np.array(grasp['grasp_point']);source[:2]+=shift
        self.confirm_held(part,0,source);robot.held=True
        if recorded_payload is None:self.measure_payload(part,allow_observer=False)
        robot.restore_loaded_gripper(0,commands[-1]['command'][7])
        observer=next((e for e in reversed(rows) if e['event']=='observer_plan'),None)
        if observer is not None:
            before=next(e for e in reversed(commands) if e['elapsed_s']<observer['elapsed_s'])
            self.observer_return=np.asarray(before['measured'][8:15])
        if robot.pose(0)[2,3]<robot.table+.13:
            payload=self.measure_payload(part,allow_observer=False)
            rotation=np.asarray(grasp['rotation']);high=max(b.rim for b in bins.values())+.11
            _,_,_,_,_,corridor,_=self.plan_place(part,bins['C'],rotation,source,high,payload_override=payload,lift_only=True)
            low=robot.pose(0)[:3,3].copy()
            for vertex in corridor:
                for fraction in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(vertex-low)/.02))+1))[1:]:
                    robot.cartesian(0,low+(vertex-low)*fraction,rotation,'clear_rims')
                low=vertex.copy()
        self.log('held_transport_resumed',checkpoint=str(checkpoint),initial_counts=self.initial_counts)
        return self.finish_placement(part,bins['C'],source,np.array(grasp['rotation']),
            max(b.rim for b in bins.values())+.11,None,None,None,None,robot.pose(0)[:3,3].copy(),[])

    @timed('verification')
    def confirm_final(self):
        """Recheck all bins together; a past placement ledger is insufficient."""
        expected=dict(self.initial_counts)
        for entry in self.completed:expected[entry['kind']]+=1
        stable=0;last_stamp=None;deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            try:
                snapshot=self.scene.read(candidate_stride=2)
            except (ValueError,RuntimeError):
                time.sleep(.2)
                continue
            stamp=float(snapshot['stamp'])
            if last_stamp is not None and stamp<=last_stamp:
                time.sleep(.2)
                continue
            last_stamp=stamp
            parts,bins=self.detector.detect(snapshot,include_bins=True)
            counts={k:sum(p.kind==k and fits_bin(p,bins[k]) for p in parts) if k in bins else None for k in expected}
            stable=stable+1 if counts==expected else 0
            self.save('final_check',snapshot,parts,bins)
            self.log('final_check',stamp=stamp,expected=expected,counts=counts,stable=stable)
            if stable>=5:return counts
            time.sleep(.2)
        raise Stop('Final five-frame all-bin verification failed')

    def pending_targets(self,parts,kind=None):
        counts=dict(self.initial_counts)
        for entry in self.completed:counts[entry['kind']]+=1
        return [p for p in parts if p.confidence>=.9 and (kind is None or p.kind==kind)
                and counts[p.kind]<2]

    def run(self,count=6,kind=None):
        initial,_,bins=self.observe('initial_occupancy')
        all_parts,bins=self.detector.detect(initial,include_bins=True)
        self.initial_counts={k:sum(p.kind==k and fits_bin(p,bins[k]) for p in all_parts) for k in 'ABC'}
        self.bin_parts={k:[p for p in all_parts if p.kind==k and fits_bin(p,bins[k])] for k in 'ABC'}
        self.log('initial_occupancy',counts=self.initial_counts)
        if self.initial_counts['C']==0 and self.release_bins['C'].cavity is not None:
            self.release_empty_cavity_reference=(self.release_bins['C'],dict(self.robot.bins))
            self.log('release_empty_cavity_reference',bin=asdict(self.release_bins['C']))
        if count==6 and any(self.initial_counts.values()):
            raise Stop('Full six-piece validation requires initially empty bins')
        for index in range(count):
            for attempt in range(2):
                s,parts,bins = self.observe('select')
                options = self.pending_targets(parts,kind)
                if not options and kind in (None,'C'):
                    # The outer C handle can be hidden by the usual left park.
                    # Move the empty arm outward, then acquire fresh RGB-D.
                    robot=self.robot;robot.side=0
                    q14=robot.measured()[np.r_[0:7,8:15]]
                    scan=np.deg2rad([30,50,10,-100,-60,0,0]);start=q14[:7].copy()
                    for fraction in np.linspace(0,1,101):
                        robot.check_clearance(robot.kin.matrix(start+(scan-start)*fraction,0),0,extra_margin=.005)
                    q14[:7]=scan;robot.joint_path(q14,'scan_outer_source')
                    s,parts,bins=self.observe('select_after_scan')
                    options=self.pending_targets(parts,kind)
                # A pairs have repeatedly passed from the preparation pose.
                # Complete them before B changes the right-arm approach and
                # the observed tracking bias. All six still need fresh visual
                # confirmation in this same scene.
                options.sort(key=lambda p:('ABC'.index(p.kind),p.center[1]))
                if not options:
                    raise Stop('No unambiguous accessible target')
                try:
                    self.pick_place(options[0],bins[options[0].kind],s)
                    break
                except EmptyGrasp:
                    self.robot.held=False
                    self.robot.gripper(self.robot.side,False,'empty_grasp_reopen')
                    self.robot.target=None
                    self.log('empty_grasp_retry',attempt=attempt+1)
                    if attempt==1: raise
