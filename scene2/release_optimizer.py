"""Continuous release search using measured payload geometry and public FK.

The optimiser proposes joint poses, never commands them. Callers must check
the open/closed hand and the complete transport route before execution.
"""
import numpy as np
from scipy.optimize import minimize
from .release import free_drop_separations, free_drop_clear, C_OPENING_PLANNING_LIMIT
from .settling import c_profile, held_fits, settling_fits, settled_corners
from .control import feedback_joint_bounds


def _release_candidates(kin, side, start, seeds, payload, handle_length, cavity,
                       rim, floor, joint_margin, held_margin, reserve, occupied=(),drop_reserve=.002,
                       reference_bias=None,settling_margin=.003,settling_target=None):
    if drop_reserve<.0003:raise ValueError('Drop reserve cannot reduce the checked planning buffer')
    if settling_margin<.003:raise ValueError('Settling margin cannot reduce clearance')
    offset, relative, half = payload
    profile = c_profile(half, handle_length)
    polygon = (np.asarray(cavity.cavity) if cavity.cavity is not None else
               np.array([[cavity.lower[0], cavity.lower[1]], [cavity.upper[0], cavity.lower[1]],
                         [cavity.upper[0], cavity.upper[1]], [cavity.lower[0], cavity.upper[1]]]))
    edges = np.roll(polygon, -1, axis=0) - polygon
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    bounds = kin.limits[side]
    lower, upper = feedback_joint_bounds(bounds,np.asarray(start),joint_margin,reference_bias)
    cache = {}
    optimize_margin=settling_target is not None
    margin_floor=settling_margin+reserve+.00005
    solver_bounds=list(zip(lower,upper))+([(margin_floor,settling_target)] if optimize_margin else [])

    def values(x):
        q=np.asarray(x[:7])
        if not np.array_equal(cache.get('x'), x):
            pose = kin.matrix(q, side)
            rotation = pose[:3, :3] @ relative
            center = pose[:3, 3] + pose[:3, :3] @ offset
            held = center + profile @ rotation.T
            # Near-vertical intermediate proposals have no bounded landing
            # model. Keep a finite penalty so the optimiser can leave them.
            axis = rotation[:, 0]
            horizontal = np.linalg.norm(axis[:2])
            direction = axis[:2] / max(horizontal, 1e-9)
            landing_offset = -np.sign(axis[2]) * (axis[:2] - direction) * half[0]
            transverse = np.array([-direction[1], direction[0]])
            settled = center[:2] + landing_offset + profile[:, 0, None] * direction + profile[:, 1, None] * transverse
            held_distances = (held[:, :2, None] - polygon.T[None, :, :]) * normals.T[None, :, :]
            settled_distances = (settled[:, :, None] - polygon.T[None, :, :]) * normals.T[None, :, :]
            bottom = held[:, 2].min()
            residual = np.r_[held_distances.sum(1).ravel() - held_margin - reserve - .00005,
                             # Preserve the original arithmetic for baseline
                             # seeds: regrouping these small subtractions can
                             # change the nonlinear solver's selected basin.
                             (settled_distances.sum(1).ravel() - x[7] if optimize_margin else
                              settled_distances.sum(1).ravel() - settling_margin - reserve - .00005),
                             bottom - rim - .020, rim + .12 - bottom,
                             .8 - abs(axis[2]),
                             C_OPENING_PLANNING_LIMIT - abs(pose[2, 0]),
                             free_drop_separations(pose, payload, handle_length, floor) - drop_reserve]
            cache.update(x=x.copy(), q=q.copy(), pose=pose, rotation=rotation, center=center, bottom=bottom, residual=residual)
        return cache

    def objective(x):
        q=x[:7]
        v = values(x)
        score = (.01 * np.sum((q - start) ** 2) + 100 * np.sum((v['center'][:2] - cavity.center) ** 2)
                 + 100 * (v['bottom'] - rim - .045) ** 2)
        if occupied:
            direction = v['rotation'][:2, 0]
            direction = direction / max(np.linalg.norm(direction), 1e-9)
            score += .1 * max(float(direction @ np.asarray(p.axis)) ** 2 for p in occupied)
        return score+(1000*(settling_target-x[7]) if optimize_margin else 0.)

    public = np.deg2rad([-30, 10 if side == 0 else -10, 10 if side == 0 else -10,
                         -80, -70 if side == 0 else 70, 0, 0])
    attempted = []
    accepted = []
    for seed in [start, *reversed(seeds), public, (lower + upper) / 2]:
        initial = np.clip(seed, lower + 1e-5, upper - 1e-5)
        if optimize_margin:initial=np.r_[initial,margin_floor]
        if any(np.linalg.norm(initial - previous) < .01 for previous in attempted):
            continue
        attempted.append(initial)
        result = minimize(objective, initial, method='SLSQP', bounds=solver_bounds,
                          constraints=[dict(type='ineq', fun=lambda x: values(x)['residual'])],
                          options=dict(maxiter=180, ftol=1e-10))
        solution=result.x
        q = solution[:7]
        v = values(solution)
        if (not np.all(np.isfinite(q)) or np.any(q < lower) or np.any(q > upper)
                or v['residual'].min() < -1e-7):
            continue
        rotation, center = v['rotation'], v['center']
        if not held_fits(cavity, center, np.abs(rotation) @ half, held_margin + reserve,
                         (profile @ rotation.T)[:, :2]):
            continue
        if not settling_fits(cavity, center, rotation[:, 0], half, margin=settling_margin + reserve, handle_length=handle_length):
            continue
        if not free_drop_clear(v['pose'], payload, handle_length, floor):
            continue
        if any(np.linalg.norm(q - previous) < .02 for previous in accepted):
            continue
        accepted.append(q.copy())
        yield objective(solution), q.copy()


def release_candidates(kin, side, start, seeds, payload, handle_length, cavity,
                       rim, floor, joint_margin, held_margin, reserve, occupied=(),drop_reserve=.002,
                       reference_bias=None,settling_margin=.003,settling_target=None):
    """Refine feasible poses toward extra landing room, preserving hard guards.

    The target is total planned clearance, including the reserve. A target
    beyond the feasible geometry must not discard all otherwise checked poses;
    live measured release checks still decide whether the hand may open.
    """
    if settling_target is not None and (not np.isfinite(settling_target) or
            settling_target<settling_margin+reserve+.00005):
        raise ValueError('Settling target cannot reduce the required clearance')
    args=(kin,side,start,seeds,payload,handle_length,cavity,rim,floor,joint_margin,held_margin,reserve,occupied)
    kw=dict(drop_reserve=drop_reserve,reference_bias=reference_bias,settling_margin=settling_margin)
    if settling_target is None:
        yield from _release_candidates(*args,**kw)
        return
    baseline=list(_release_candidates(*args,**kw))
    refined=list(args);refined[3]=list(seeds)+[q for _,q in baseline]
    yield from _release_candidates(*refined,**kw,settling_target=settling_target)
    polygon=(np.asarray(cavity.cavity) if cavity.cavity is not None else
             np.array([[cavity.lower[0],cavity.lower[1]],[cavity.upper[0],cavity.lower[1]],
                       [cavity.upper[0],cavity.upper[1]],[cavity.lower[0],cavity.upper[1]]]))
    edges=np.roll(polygon,-1,axis=0)-polygon
    normals=np.column_stack((-edges[:,1],edges[:,0]))
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    for score,q in baseline:
        pose=kin.matrix(q,side)
        center=pose[:3,3]+pose[:3,:3]@payload[0]
        axis=(pose[:3,:3]@payload[1])[:,0]
        corners=settled_corners(center,axis,payload[2],handle_length)
        clearance=np.min(np.sum((corners[:,None,:]-polygon[None,:,:])*normals[None,:,:],axis=2))
        yield score+1000*max(0.,settling_target-clearance),q
