"""Separate overlapping red handles only with two independently observed shafts."""
import numpy as np
import cv2


def screwdriver_pair(snapshot,bin):
    from .perception import point_image,masks,Part
    points,valid=point_image(snapshot);colours,_=masks(snapshot['rgb'])
    roi=valid&(points[:,:,2]>bin.floor+.002)&(points[:,:,2]<bin.rim+.12)
    for j in (0,1):roi&=(points[:,:,j]>bin.lower[j]-.025)&(points[:,:,j]<bin.upper[j]+.025)
    red_mask=roi&colours['C'];red=points[red_mask];metal=points[roi&colours['A']]
    if len(red)<40 or len(metal)<40:return []
    rng=np.random.default_rng(34);axes=[];remaining=metal.copy()
    for _ in range(2):
        if len(remaining)<20:return []
        best=None
        for _ in range(1500):
            a,b=remaining[rng.choice(len(remaining),2,replace=False)]
            direction=b-a;length=np.linalg.norm(direction)
            if length<.04:continue
            direction/=length
            inliers=remaining[np.linalg.norm(np.cross(remaining-a,direction),axis=1)<.0028]
            if len(inliers)<20:continue
            span=np.ptp(inliers@direction)
            if span<.065:continue
            score=len(inliers)*span
            if best is None or score>best[0]:best=(score,inliers)
        if best is None:return []
        shaft=best[1];_,basis=np.linalg.eigh(np.cov(shaft.T))
        axis=basis[:,-1];origin=np.median(shaft,axis=0)
        axes.append((origin,axis,shaft))
        remaining=remaining[np.linalg.norm(np.cross(remaining-origin,axis),axis=1)>.0045]
    # Two edges of one shaft do not provide evidence for two objects.
    for i,j in ((0,1),(1,0)):
        origin,axis,_=axes[i]
        if np.median(np.linalg.norm(np.cross(axes[j][2]-origin,axis),axis=1))<.012:return []
    distances=np.column_stack([np.linalg.norm(np.cross(red-origin,axis),axis=1) for origin,axis,_ in axes])
    eligible=[]
    for i,(origin,axis,shaft) in enumerate(axes):
        # A handle belongs at an end of its shaft, not anywhere near the
        # infinite line. Nearby tilted rods otherwise steal each other's red
        # points and appear to have one impossibly long handle.
        low,high=np.percentile(shaft@axis,[1,99]);along=red@axis
        ends=[(along>=low-.115)&(along<=low+.015),
              (along>=high-.015)&(along<=high+.115)]
        ends=[end&(distances[:,i]<.018) for end in ends]
        eligible.append(max(ends,key=np.count_nonzero))
    eligible=np.column_stack(eligible)
    # Do not discard a protruding/unsupported portion to manufacture a fit.
    if np.mean(eligible.any(axis=1))<.9:return []
    distances=np.where(eligible,distances,np.inf)
    assignment=distances.argmin(1);pixels=np.column_stack(np.nonzero(red_mask))
    result=[];handle_centers=[]
    for i,(origin,axis,shaft) in enumerate(axes):
        selected=(assignment==i)&(distances[:,i]<.018)
        handle=red[selected]
        if len(handle)<20:return []
        handle_span=np.ptp(np.percentile(handle@axis,[1,99]))
        if not .040<handle_span<.115:return []
        handle_center=np.median(handle,axis=0);handle_centers.append(handle_center)
        if np.dot(axis,np.median(shaft,axis=0)-handle_center)<0:axis=-axis
        # Each metal line must extend beyond its associated handle on one side.
        shaft_along=(shaft-handle_center)@axis
        if np.percentile(shaft_along,90)<handle_span/2+.055:return []
        entire=np.vstack((handle,shaft))
        if not .13<np.ptp(np.percentile(entire@axis,[1,99]))<.28:return []
        if np.linalg.norm(axis[:2])<.15:return []
        xy_axis=axis[:2]/np.linalg.norm(axis[:2]);basis=np.column_stack((xy_axis,[-xy_axis[1],xy_axis[0]]))
        lo,hi=np.percentile(handle[:,:2]@basis,[1,99],axis=0)
        full_lo,full_hi=np.percentile(entire[:,:2]@basis,[1,99],axis=0)
        center=np.r_[((lo+hi)/2)@basis.T,handle_center[2]]
        full_center=np.r_[((full_lo+full_hi)/2)@basis.T,np.median(entire[:,2])]
        uv=pixels[selected];lower=uv.min(0);upper=uv.max(0)
        result.append(Part('C',center.tolist(),xy_axis.tolist(),(hi-lo).tolist(),float(np.percentile(entire[:,2],95)),
            [int(lower[1]),int(lower[0]),int(upper[1]),int(upper[0])],float(snapshot['stamp']),1.,
            full_center.tolist(),(full_hi-full_lo).tolist(),cv2.convexHull(entire[:,:2].astype(np.float32)).reshape(-1,2).tolist()))
    if np.linalg.norm(handle_centers[0]-handle_centers[1])<.015:return []
    return result
