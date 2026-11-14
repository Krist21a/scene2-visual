"""In-hand screwdriver pose from calibrated RGB-D, without scene state."""
import numpy as np
from .perception import point_image,masks


def source_payload(part,tcp,table):
    """Full source footprint, with C's positive axis always toward its tip."""
    axis=np.asarray(part.axis)
    center=np.asarray(part.full_center or part.center)
    if part.kind=='C' and np.dot(center[:2]-np.asarray(part.center)[:2],axis)<0:
        axis=-axis
    basis=np.column_stack((np.r_[axis,0],np.r_[[-axis[1],axis[0]],0],[0,0,1]))
    half=np.r_[part.full_extent or part.extent,part.top-table]/2
    if part.kind=='C':
        # Match the hidden round surface and endpoint reserve used by the
        # post-lift estimator, rather than validating an optimistic grasp.
        half[0]+=.001
        half[1:]=np.maximum(half[1:],.012)
    return tcp[:3,:3].T@(center-tcp[:3,3]),tcp[:3,:3].T@basis,half


def screwdriver_payload(snapshot,tcp,prior_length=None):
    points,valid=point_image(snapshot)
    colours,_=masks(snapshot['rgb'])
    table=float(snapshot['table_z_base_m'])
    distance=np.linalg.norm(points-tcp[:3,3],axis=-1)
    red=valid&colours['C']&(distance<.12)&(points[:,:,2]>table+.045)
    handle=points[red]
    if len(handle)<30:raise ValueError('Insufficient elevated handle points')
    center=np.median(handle,axis=0)
    _,vectors=np.linalg.eigh(np.cov(handle.T));axis=vectors[:,-1]
    if np.dot(axis,center-tcp[:3,3])<0:axis=-axis
    along=(points-center)@axis
    perpendicular=np.linalg.norm(points-center-along[:,:,None]*axis,axis=-1)
    hlo,hhi=np.percentile((handle-center)@axis,[1,99])
    if not .035<hhi-hlo<.115:raise ValueError('Incomplete in-hand handle')
    outside_handle=((along>hhi+.003)&(along<hhi+.18))|((along<hlo-.003)&(along>hlo-.18))
    metal=(valid&colours['A']&(distance>.035)&(distance<.26)&(perpendicular<.04)
           &(points[:,:,2]>table+.025)&outside_handle)
    candidates=points[metal]
    if len(candidates)<20:raise ValueError('Metal shaft not visible in hand')
    rng=np.random.default_rng(421);best=None
    for _ in range(250):
        a,b=candidates[rng.choice(len(candidates),2,replace=False)]
        direction=b-a;length=np.linalg.norm(direction)
        if length<.05:continue
        direction/=length
        if np.dot(direction,axis)<0:direction=-direction
        if np.dot(direction,axis)<.82:continue
        if np.linalg.norm(np.cross(center-a,direction))>.02:continue
        distances=np.linalg.norm(np.cross(candidates-a,direction),axis=1)
        inliers=candidates[distances<.004]
        if len(inliers)<20:continue
        span=np.ptp(inliers@direction)
        if span<.065:continue
        score=len(inliers)*span
        if best is None or score>best[0]:best=(score,inliers,direction)
    if best is None:raise ValueError('No supported metal shaft line')
    _,shaft,axis=best
    _,vectors=np.linalg.eigh(np.cov(shaft.T));refined=vectors[:,-1]
    if np.dot(refined,axis)<0:refined=-refined
    axis=refined
    # A grasp near the shaft puts the handle centre behind the TCP. The
    # observed metal, not that grasp offset, determines the positive end.
    if np.dot(np.mean(shaft,axis=0)-center,axis)<0:axis=-axis
    vertical=np.array([0.,0.,1.])
    width=np.cross(vertical,axis)
    if np.linalg.norm(width)<.1:width=np.cross([0.,1.,0.],axis)
    width/=np.linalg.norm(width)
    basis=np.column_stack((axis,width,np.cross(axis,width)))
    entire=np.vstack((handle,shaft));local=(entire-center)@basis
    lo,hi=np.percentile(local,[1,99],axis=0)
    # Estimate each material's axial endpoints separately. The dense handle
    # must not outvote the sparse, RANSAC-supported far tip. A millimetre
    # endpoint buffer covers the remaining sub-percentile tail.
    handle_ends=np.percentile((handle-center)@axis,[.1,99.9])
    shaft_ends=np.percentile((shaft-center)@axis,[.1,99.9])
    lo[0]=min(handle_ends[0],shaft_ends[0])-.001
    hi[0]=max(handle_ends[1],shaft_ends[1])+.001
    extent=hi-lo
    if not (.13<extent[0]<.28 and np.all(extent[1:]<.045)):
        raise ValueError(f'In-hand full screwdriver shape inconsistent: {extent}, shaft {len(shaft)}')
    if prior_length is not None:
        if extent[0]>prior_length+.03:raise ValueError('In-hand shaft exceeds observed source length')
        # Shaft points locate the far tip; preserve the source observation's
        # total length when the hand occludes the back of the handle.
        lo[0]=min(lo[0],hi[0]-prior_length)
        extent=hi-lo
    full_center=center+basis@((lo+hi)/2)
    # Round handle envelope: retain its hidden back surface conservatively.
    half=extent/2
    half[1:]=np.maximum(half[1:],.012)
    payload=(tcp[:3,:3].T@(full_center-tcp[:3,3]),tcp[:3,:3].T@basis,half)
    return payload,dict(handle_points=len(handle),shaft_points=len(shaft),center=full_center,
                        axis=axis,extent=2*half,stamp=float(snapshot['stamp']))
