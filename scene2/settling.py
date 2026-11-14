"""Geometric prediction of a tilted rod settling about its lowest endpoint.

This is a planning approximation, never a substitute for post-release RGB-D
verification. No simulator object states or physical parameters are read.
"""
import numpy as np

def support_bias(region, occupied):
    """Observed rearward deflection when a second rod lands on the first.

    The 30 mm correction relative to the free-tip model comes from registered
    RGB-D release/settled pairs. Final vision still determines acceptance.
    """
    if not occupied:return np.zeros(2)
    direction=(np.asarray(region.cavity[1])-region.cavity[0] if region.cavity is not None else np.array([1.,0.]))
    direction=direction/np.linalg.norm(direction)
    if direction[0]<0:direction=-direction
    return .03*direction

def held_fits(region, center, extent, margin, offsets=None):
    center=np.asarray(center)[:2];extent=np.asarray(extent)[:2]
    if offsets is not None:return all(region.contains(center+point,margin) for point in offsets)
    return all(region.contains(center+extent*[x,y],margin) for x,y in ((-1,-1),(1,-1),(1,1),(-1,1)))

def c_profile(half, handle_length):
    """The same observed handle/thin-shaft profile used by motion guards."""
    length=2*half[0];handle=min(handle_length,length)
    points=[]
    for lo,hi,radius in ((-half[0],-half[0]+handle,max(half[1:])),(-half[0]+handle,half[0],.004)):
        points.extend([[x,y*radius,z*radius] for x in (lo,hi) for y in (-1,1) for z in (-1,1)])
    return np.asarray(points)

def settling_offset(axis, half_length):
    axis=np.asarray(axis,dtype=float)
    axis=axis/np.linalg.norm(axis)
    horizontal=np.linalg.norm(axis[:2])
    if horizontal<.2:raise ValueError('Rod too vertical for a bounded settling prediction')
    direction=axis[:2]/horizontal
    lowest=-np.sign(axis[2])
    return lowest*(axis[:2]-direction)*half_length, direction

def settled_corners(center, axis, half, handle_length=None,bias=None):
    offset,direction=settling_offset(axis,float(half[0]))
    if bias is not None:offset=offset+np.asarray(bias)
    transverse=np.array([-direction[1],direction[0]])
    radius=float(max(half[1:]))
    if handle_length is not None:
        points=c_profile(half,handle_length)
        return np.asarray(center)[:2]+offset+points[:,0,None]*direction+points[:,1,None]*transverse
    return np.asarray(center)[:2]+offset+np.array([
        a*half[0]*direction+b*radius*transverse for a,b in ((-1,-1),(1,-1),(1,1),(-1,1))])

def release_center(visible, cavity, axis, half, extent, margin, handle_length=None, held_rotation=None,bias=None,reserve=0.,settling_margin=.003):
    if reserve<0:raise ValueError('Release reserve cannot reduce clearance')
    if settling_margin<.003:raise ValueError('Settling margin cannot reduce clearance')
    margin=margin+reserve
    settled_margin=settling_margin+reserve
    offset,_=settling_offset(axis,float(half[0]))
    if bias is not None:offset=offset+np.asarray(bias)
    held_offsets=(c_profile(half,handle_length)@held_rotation.T)[:,:2] if handle_length is not None and held_rotation is not None else None
    lower=np.asarray(visible.lower)-(held_offsets.min(0) if held_offsets is not None else -np.asarray(extent)[:2])+margin
    upper=np.asarray(visible.upper)-(held_offsets.max(0) if held_offsets is not None else np.asarray(extent)[:2])-margin
    if np.any(lower>upper):raise ValueError('Held envelope does not fit release region')
    cavity_polygon=(np.asarray(cavity.cavity) if cavity.cavity is not None else
                    np.array([[cavity.lower[0],cavity.lower[1]],[cavity.upper[0],cavity.lower[1]],
                              [cavity.upper[0],cavity.upper[1]],[cavity.lower[0],cavity.upper[1]]]))
    polygon=np.array([[lower[0],lower[1]],[upper[0],lower[1]],
                      [upper[0],upper[1]],[lower[0],upper[1]]])
    corners=settled_corners([0,0,0],axis,half,handle_length,bias)
    # Intersect the held-AABB region with all translated settled-footprint
    # half-planes. Independent XY clipping can miss a feasible diagonal shift.
    for point,end in zip(cavity_polygon,np.roll(cavity_polygon,-1,axis=0)):
        edge=end-point;normal=np.array([-edge[1],edge[0]])/np.linalg.norm(edge)
        bound=settled_margin+normal@point-np.min(corners@normal)
        if visible.cavity is not None:
            held_min=np.min(held_offsets@normal) if held_offsets is not None else -np.abs(normal)@np.asarray(extent)[:2]
            bound=max(bound,margin+normal@point-held_min)
        clipped=[]
        for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
            da,db=normal@a-bound,normal@b-bound
            if da>=0:clipped.append(a)
            if (da>=0)!=(db>=0):clipped.append(a+(b-a)*da/(da-db))
        if not clipped:raise ValueError('No common held and settled release region')
        polygon=np.asarray(clipped)
    target=cavity_polygon.mean(0)-offset
    if (np.all(target>=lower) and np.all(target<=upper) and held_fits(visible,target,extent,margin,held_offsets) and
            settling_fits(cavity,np.r_[target,0],axis,half,margin=settled_margin,handle_length=handle_length,bias=bias)):return target
    candidates=[]
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        edge=b-a;length=edge@edge
        candidates.append(a+edge*np.clip((target-a)@edge/length,0,1) if length>1e-18 else a)
    # Move very slightly inside to avoid rounding a boundary point outside.
    nearest=min(candidates,key=lambda p:np.linalg.norm(p-target))
    return nearest*(1-1e-6)+polygon.mean(0)*1e-6

def settling_fits(cavity, center, axis, half, margin=.003,handle_length=None,bias=None):
    try:return all(cavity.contains(point,margin) for point in settled_corners(center,axis,half,handle_length,bias))
    except ValueError:return False

