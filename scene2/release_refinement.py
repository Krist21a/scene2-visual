"""Improve a feasible release's held clearance without losing landing room.

This only proposes a pose. The caller must recheck the complete open/closed
hand and transport route, retaining its original feasible plan on failure.
"""
import numpy as np
from scipy.optimize import minimize
from .control import feedback_joint_bounds
from .release import free_drop_separations, free_drop_clear, C_OPENING_PLANNING_LIMIT
from .settling import c_profile, held_fits, settling_fits


def refine_held_clearance(kin, side, start, selected, payload, handle_length,
                          cavity, rim, floor, joint_margin, reference_bias=None,
                          drop_reserve=.002, gain=.001):
    offset, relative, half=payload
    profile=c_profile(half,handle_length)
    polygon=(np.asarray(cavity.cavity) if cavity.cavity is not None else
             np.array([[cavity.lower[0],cavity.lower[1]],[cavity.upper[0],cavity.lower[1]],
                       [cavity.upper[0],cavity.upper[1]],[cavity.lower[0],cavity.upper[1]]]))
    edges=np.roll(polygon,-1,axis=0)-polygon
    normals=np.column_stack((-edges[:,1],edges[:,0]))
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    lower,upper=feedback_joint_bounds(kin.limits[side],start,joint_margin,reference_bias)

    def geometry(q):
        pose=kin.matrix(q,side);rotation=pose[:3,:3]@relative
        center=pose[:3,3]+pose[:3,:3]@offset
        held=center+profile@rotation.T
        axis=rotation[:,0]
        direction=axis[:2]/max(np.linalg.norm(axis[:2]),1e-9)
        landing_offset=-np.sign(axis[2])*(axis[:2]-direction)*half[0]
        transverse=np.array([-direction[1],direction[0]])
        # Finite even for rejected near-vertical solver proposals. The axis
        # constraint below and final independent guard reject those poses.
        settled=center[:2]+landing_offset+profile[:,0,None]*direction+profile[:,1,None]*transverse
        hd=((held[:,:2,None]-polygon.T[None,:,:])*normals.T[None,:,:]).sum(1).ravel()
        sd=((settled[:,:,None]-polygon.T[None,:,:])*normals.T[None,:,:]).sum(1).ravel()
        return pose,rotation,center,held[:,2].min(),hd,sd

    original=geometry(selected)
    held_before=float(original[4].min());landing_before=float(original[5].min())
    target=held_before+gain
    cache={}
    def values(x):
        if not np.array_equal(cache.get('x'),x):
            pose,rotation,center,bottom,hd,sd=geometry(x[:7])
            residual=np.r_[hd-x[7],sd-landing_before,bottom-rim-.020,rim+.12-bottom,
                           .8-abs(rotation[2,0]),C_OPENING_PLANNING_LIMIT-abs(pose[2,0]),
                           free_drop_separations(pose,payload,handle_length,floor)-drop_reserve]
            cache.update(x=x.copy(),pose=pose,rotation=rotation,center=center,
                         residual=residual,held=float(hd.min()),landing=float(sd.min()))
        return cache

    initial=np.r_[np.clip(selected,lower+1e-5,upper-1e-5),held_before-1e-8]
    result=minimize(lambda x:1000*(target-x[7])+.01*np.sum((x[:7]-selected)**2),initial,
                    method='SLSQP',bounds=list(zip(lower,upper))+[(held_before-1e-8,target)],
                    # Search slightly inside the constraints. C47's optimum
                    # otherwise missed the original numerical check by 0.24um;
                    # the checked tolerances themselves remain unchanged.
                    constraints=[dict(type='ineq',fun=lambda x:values(x)['residual']-1e-6)],
                    options=dict(maxiter=180,ftol=1e-10))
    q=result.x[:7];v=values(result.x)
    if (not np.all(np.isfinite(result.x)) or np.any(q<lower) or np.any(q>upper)
            or v['residual'].min()<-1e-7 or v['held']<held_before+.00005
            or v['landing']<landing_before-1e-7):
        return None
    if not held_fits(cavity,v['center'],np.abs(v['rotation'])@half,held_before-1e-7,
                     (profile@v['rotation'].T)[:,:2]):return None
    if not settling_fits(cavity,v['center'],v['rotation'][:,0],half,
                         margin=landing_before-1e-7,handle_length=handle_length):return None
    if not free_drop_clear(v['pose'],payload,handle_length,floor):return None
    return q.copy(),dict(held_before_m=held_before,held_after_m=v['held'],
                         settling_before_m=landing_before,settling_after_m=v['landing'])
