import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from scene2.control import rotation_log_right_inverse


class PoseDerivativeTests(unittest.TestCase):
    def test_rotation_residual_against_central_differences(self):
        rng=np.random.default_rng(731)
        for angle in (0.,1e-7,.3,1.8,3.05):
            axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
            error=axis*angle
            relative=Rotation.from_rotvec(error)
            jac=rng.normal(size=(3,7))
            exact=-rotation_log_right_inverse(error)@jac
            numeric=np.empty((3,7));h=1e-6
            for j in range(7):
                plus=(relative*Rotation.from_rotvec(-h*jac[:,j])).as_rotvec()
                minus=(relative*Rotation.from_rotvec(h*jac[:,j])).as_rotvec()
                numeric[:,j]=(plus-minus)/(2*h)
            np.testing.assert_allclose(exact,numeric,atol=1e-8,rtol=1e-7)
