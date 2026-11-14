"""Check a gravity release against public open-hand geometry and RGB-D payload.

The swept prisms deliberately overestimate a falling part. Passing this check
only establishes clearance from the stationary open hand, not landing success.
"""
import numpy as np
from scipy.spatial import ConvexHull
from .control import open_finger_geometry, prism_overlap

C_OPENING_VERTICAL_LIMIT = .15
C_OPENING_PLANNING_LIMIT = C_OPENING_VERTICAL_LIMIT - .01


def side_opening_clear(pose):
    """Keep opposing pads at nearly equal heights as they separate.

    The final open-hand sweep alone cannot rule out transient support by a
    lower pad during opening. Local X is the calibrated jaw closing axis.
    This guard supplements the geometric sweep; it is not a landing model.
    """
    return bool(np.isfinite(pose[2, 0]) and abs(pose[2, 0]) <= C_OPENING_VERTICAL_LIMIT)


def free_drop_separations(pose, payload, handle_length, floor, margin=.003):
    """Signed SAT separation for each falling segment / open-hand pair.

    Positive values separate the same conservative prisms as free_drop_clear.
    These distances guide optimisation; the original guard still authorises
    the final pose independently.
    """
    offset, relative, half = payload
    rotation = pose[:3, :3] @ relative
    center = pose[:3, 3] + pose[:3, :3] @ offset
    handle = min(handle_length, 2 * half[0])
    segments = [(-(2 * half[0] - handle) / 2, np.array([handle / 2, *half[1:]])),
                (handle / 2, np.array([(2 * half[0] - handle) / 2, .004, .004]))]
    hand = open_finger_geometry() + [(np.array([0., 0., .105]), np.array([.045, .035, .035]))]
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    vertical = np.array([0., 0., 1.])
    wrist = pose[:3, :3]
    result = []
    for shift, size in segments:
        if size[0] <= 0:
            continue
        vertices = center + rotation[:, 0] * shift + (signs * (size + margin)) @ rotation.T
        xy = vertices[ConvexHull(vertices[:, :2]).vertices, :2]
        prism = np.vstack((np.column_stack((xy, np.full(len(xy), floor))),
                           np.column_stack((xy, np.full(len(xy), vertices[:, 2].max())))))
        edges = np.column_stack((np.roll(xy, -1, axis=0) - xy, np.zeros(len(xy))))
        axes = np.vstack(([vertical], wrist.T,
                          np.cross(np.vstack(([vertical], edges))[:, None, :], wrist.T[None, :, :]).reshape(-1, 3),
                          np.cross(edges, vertical)))
        axes = axes[np.linalg.norm(axes, axis=1) > 1e-8]
        axes /= np.linalg.norm(axes, axis=1)[:, None]
        projected = prism @ axes.T
        lower, upper = projected.min(0), projected.max(0)
        for hand_offset, hand_half in hand:
            middle = (pose[:3, 3] + wrist @ hand_offset) @ axes.T
            radius = np.abs(axes @ wrist) @ hand_half
            result.append(np.maximum(lower - middle - radius, middle - radius - upper).max())
    return np.asarray(result)


def free_drop_clear(pose, payload, handle_length, floor, margin=.003):
    offset, relative, half = payload
    rotation = pose[:3, :3] @ relative
    center = pose[:3, 3] + pose[:3, :3] @ offset
    handle = min(handle_length, 2 * half[0])
    segments = [(-(2 * half[0] - handle) / 2, np.array([handle / 2, *half[1:]])),
                (handle / 2, np.array([(2 * half[0] - handle) / 2, .004, .004]))]
    hand = open_finger_geometry() + [(np.array([0., 0., .105]), np.array([.045, .035, .035]))]
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    for shift, size in segments:
        if size[0] <= 0:
            continue
        vertices = center + rotation[:, 0] * shift + (signs * (size + margin)) @ rotation.T
        footprint = vertices[ConvexHull(vertices[:, :2]).vertices, :2]
        for hand_offset, hand_half in hand:
            hand_center = pose[:3, 3] + pose[:3, :3] @ hand_offset
            if prism_overlap(hand_center, pose[:3, :3], hand_half,
                             footprint, floor, vertices[:, 2].max()):
                return False
    return True
