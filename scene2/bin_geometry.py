"""Conservative cavity bounds from the observed coloured bin silhouette.

Used for settled-object verification: the front lip hides part of the floor.
No scene poses or layout are used. Motion planning retains visible-floor bounds.
"""
from dataclasses import replace
import cv2
import numpy as np
from scipy.optimize import least_squares


def propagate_cavity(reference, reference_bins, current_bins):
    """Carry an observed cavity through brief occlusion with two references.

    Returns None when the other bins do not support a coherent translation.
    The original observation remains the anchor, avoiding accumulated drift.
    """
    keys=[k for k in reference_bins if k!=reference.kind and k in current_bins]
    if reference.cavity is None or len(keys)<2:return None
    shifts=np.array([np.asarray(current_bins[k].center)-reference_bins[k].center for k in keys])
    shift=np.mean(shifts,axis=0)
    residual=float(np.max(np.linalg.norm(shifts-shift,axis=1)))
    if residual>.003 or np.linalg.norm(shift)>.03:return None
    polygon=np.asarray(reference.cavity)+shift
    observed_polygon=polygon.copy()
    center=polygon.mean(0)
    # Reserve registration uncertainty in addition to the original fit inset.
    half=np.min(np.linalg.norm(np.roll(polygon,-1,axis=0)-polygon,axis=1))/2
    polygon=center+(polygon-center)*(1-(residual+.002)/half)
    visible=current_bins[reference.kind]
    result=replace(visible,center=center.tolist(),lower=polygon.min(0).tolist(),upper=polygon.max(0).tolist(),cavity=polygon.tolist())
    # Floor bounds are independent coordinate extrema, not four observed
    # corners (a rotated floor need not contain its bounding-box corners).
    if (np.any(np.asarray(visible.lower)<observed_polygon.min(0)-.002) or
            np.any(np.asarray(visible.upper)>observed_polygon.max(0)+.002) or
            not result.contains(visible.center)):return None
    return result


def register_cavity(snapshot, colour, visible, reference, reference_bins, current_bins):
    """Refit a previously observed rigid box through a small foreground occlusion.

    Transparent walls provide no usable depth. Depth is used only to establish
    that an opaque foreground surface hides a predicted silhouette corner.
    Width/depth retain the known-empty measurement; current colour support
    determines translation, yaw and wall heights. The original 15 mm inset and
    silhouette residual limits remain in force.
    """
    if snapshot.get('depth_m') is None or propagate_cavity(reference,reference_bins,current_bins) is None:
        return None
    count,labels,stats,_=cv2.connectedComponentsWithStats(colour.astype(np.uint8),8)
    if count<2:return None
    index=1+np.argmax(stats[1:,cv2.CC_STAT_AREA])
    if stats[index,cv2.CC_STAT_AREA]<500:return None
    yy,xx=np.nonzero(labels==index)
    hull=cv2.convexHull(np.column_stack((xx,yy)).astype(np.float32)).reshape(-1,2)
    angles=np.arange(72)*np.pi/36
    directions=np.column_stack((np.cos(angles),np.sin(angles)))
    support=np.max(hull@directions.T,axis=0)
    camera=np.linalg.inv(snapshot['camera_to_base']);intrinsic=snapshot['intrinsic']
    table=float(snapshot['table_z_base_m']);depth=np.asarray(snapshot['depth_m'])
    polygon=np.asarray(reference.cavity)
    axis=polygon[1]-polygon[0]
    width=np.linalg.norm(axis)+.030;length=np.linalg.norm(polygon[3]-polygon[0])+.030
    yaw=np.arctan2(axis[1],axis[0])
    keys=[k for k in reference_bins if k!=reference.kind and k in current_bins]
    shift=np.mean([np.asarray(current_bins[k].center)-reference_bins[k].center for k in keys],axis=0)
    center=polygon.mean(0)+shift
    lower,upper=np.asarray(visible.lower),np.asarray(visible.upper)

    def evaluate(parameters):
        x,y,angle,front,back=parameters
        c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s],[s,c]])
        corners=np.array([[a*width/2,b*length/2] for a in (-1,1) for b in (-1,1)])@rotation.T+[x,y]
        points=np.vstack((np.column_stack((corners,np.full(4,table))),
                          np.column_stack((corners,table+np.array([front,front,back,back])))))
        projected=points@camera[:3,:3].T+camera[:3,3]
        pixels=projected@intrinsic.T;pixels=pixels[:,:2]/pixels[:,2,None]
        extreme=(pixels@directions.T).argmax(0)
        difference=np.max(pixels@directions.T,axis=0)-support
        hidden=[];in_frame=[]
        for pixel,z in zip(pixels,projected[:,2]):
            u,v=np.round(pixel).astype(int)
            inside=2<=u<depth.shape[1]-2 and 2<=v<depth.shape[0]-2
            in_frame.append(inside and z>.1)
            patch=depth[v-2:v+3,u-2:u+3] if inside else np.empty((0,0))
            hidden.append(inside and np.count_nonzero((patch>.1)&(patch<z-.020))>=18)
        occluded=np.asarray(hidden)[extreme]
        # Missing colour is excused only where measured foreground depth
        # supports the occlusion. Observed colour outside the model is never
        # excused, and unobserved image borders cannot justify an extension.
        residual=np.where(occluded&(difference>0),0.,difference)
        floor=np.r_[np.maximum(0,np.array([x-width/2,y-length/2])+.008-lower)*2000,
                    np.maximum(0,upper-np.array([x+width/2,y+length/2])+.008)*2000]
        return np.r_[residual,floor],occluded,in_frame

    fit=least_squares(lambda p:evaluate(p)[0],[*center,yaw,.03,.15],
                      bounds=([*(center-.006),yaw-.05,0.,.04],
                              [*(center+.006),yaw+.05,.16,.20]),loss='soft_l1',max_nfev=300)
    residual,occluded,in_frame=evaluate(fit.x)
    error=np.abs(residual[:72])
    if (not np.isfinite(fit.x).all() or not all(in_frame) or occluded.sum()>18
            or np.median(error)>1 or np.percentile(error,90)>3 or error.max()>12
            or np.max(np.abs(residual[72:]))>1):return None
    x,y,angle,_,_=fit.x
    c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s],[s,c]])
    cavity=(np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*(np.array([width,length])/2-.015))@rotation.T+[x,y]
    result=replace(visible,center=[x,y],lower=cavity.min(0).tolist(),upper=cavity.max(0).tolist(),cavity=cavity.tolist())
    if not result.contains(visible.center):return None
    return result,dict(reference_stamp=reference.stamp,occluded_directions=int(occluded.sum()),
                       median_error_px=float(np.median(error)),p90_error_px=float(np.percentile(error,90)),
                       max_error_px=float(error.max()),parameters=fit.x.tolist(),outer_size=[width,length])


def cavity_bounds(snapshot,colour,visible):
    n,labels,stats,_=cv2.connectedComponentsWithStats(colour.astype(np.uint8),8)
    if n<2:return visible
    index=1+np.argmax(stats[1:,cv2.CC_STAT_AREA])
    if stats[index,cv2.CC_STAT_AREA]<500:return visible
    yy,xx=np.nonzero(labels==index)
    hull=cv2.convexHull(np.column_stack((xx,yy)).astype(np.float32)).reshape(-1,2)
    angles=np.arange(72)*np.pi/36
    directions=np.column_stack((np.cos(angles),np.sin(angles)))
    support=np.max(hull@directions.T,axis=0)
    camera=np.linalg.inv(snapshot['camera_to_base']);intrinsic=snapshot['intrinsic']
    table=float(snapshot['table_z_base_m']);center=np.array(visible.center)
    lo,hi=np.array(visible.lower),np.array(visible.upper)
    def residual(parameters):
        x,y,width,depth,yaw,front,back=parameters
        c,s=np.cos(yaw),np.sin(yaw);rotation=np.array([[c,-s],[s,c]])
        corners=np.array([[a*width/2,b*depth/2] for a in (-1,1) for b in (-1,1)])@rotation.T+[x,y]
        points=np.vstack((np.column_stack((corners,np.full(4,table))),
                          np.column_stack((corners,table+np.array([front,front,back,back])))))
        projected=(points@camera[:3,:3].T+camera[:3,3])@intrinsic.T
        pixels=projected[:,:2]/projected[:,2,None]
        # Every observed floor point must also be inside the fitted outer box.
        return np.r_[np.max(pixels@directions.T,axis=0)-support,
                     np.maximum(0,np.array([x-width/2,y-depth/2])+.008-lo)*2000,
                     np.maximum(0,hi-np.array([x+width/2,y+depth/2])+.008)*2000]
    fit=least_squares(residual,[*center,.23,.23,0,.03,.15],
        bounds=([*(center-.10),.15,.15,-.10,0.,.04],
                [*(center+.10),.35,.35,.10,.16,.20]),loss='soft_l1',max_nfev=300)
    error=np.abs(residual(fit.x)[:72])
    if np.median(error)>1 or np.percentile(error,90)>3 or error.max()>12:return visible
    x,y,width,depth,yaw,_,_=fit.x
    # Preserve the fitted orientation for verification. The inscribed AABB
    # below remains a conservative ROI, but cuts off valid cavity corners.
    half=np.array([width,depth])/2-.015
    c,s=np.cos(yaw),np.sin(yaw)
    rotation=np.array([[c,-s],[s,c]])
    cavity=(np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*half)@rotation.T+[x,y]
    c,s=abs(np.cos(yaw)),abs(np.sin(yaw))
    half*=min(half/(np.array([[c,s],[s,c]])@half))
    lower=np.minimum(lo,np.array([x,y])-half)
    upper=np.maximum(hi,np.array([x,y])+half)
    if np.max(np.r_[lo-lower,upper-hi])>.04:return visible
    return replace(visible,center=((lower+upper)/2).tolist(),lower=lower.tolist(),upper=upper.tolist(),cavity=cavity.tolist())
