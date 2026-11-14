"""Match an image to public joint feedback before computing tool-relative geometry."""
import numpy as np


def image_arm_pose(scene, kin, stamp, side, max_dt=.035):
    with scene.lock:
        history = list(scene.joint_history)
    if not history or not np.isfinite(stamp):
        raise ValueError('No valid image/arm timestamp history')
    joint = min(history, key=lambda m: abs(m.header.stamp.to_sec()-stamp))
    dt = abs(joint.header.stamp.to_sec()-stamp)
    if dt > max_dt:
        raise ValueError('Image/arm timestamps mismatch')
    q = np.asarray(joint.joint_data.joint_q, dtype=float)[13+7*side:20+7*side]
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError('Invalid synchronized arm feedback')
    return kin.matrix(q, side), dt
