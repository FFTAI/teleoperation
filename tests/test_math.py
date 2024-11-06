import numpy as np
from scipy.spatial.transform import Rotation as R

from teleoperation.utils import (
    R_to_ortho6d,
    ortho6d_to_R,
    ortho6d_to_so3,
    se3_to_xyzortho6d,
    so3_to_ortho6d,
    xyzortho6d_to_se3,
)


def ortho6d_conversion():
    # random rotation matrix
    rot = R.from_euler("xyz", np.random.rand(3)).as_matrix()

    # convert to continuous 6D representation
    ortho6d = so3_to_ortho6d(rot)

    # convert back to SO(3)
    rot_recovered = ortho6d_to_so3(ortho6d)

    # check if the recovered rotation matrix is close to the original
    assert np.allclose(rot, rot_recovered), "Failed to recover rotation matrix from continuous 6D representation"


def xyzortho6d_conversion():
    # random SE(3) matrix
    se3 = np.eye(4)
    se3[:3, :3] = R.from_euler("xyz", np.random.rand(3)).as_matrix()
    se3[:3, 3] = np.random.rand(3)

    # convert to continuous 6D representation
    xyzortho6d = se3_to_xyzortho6d(se3)

    # convert back to SE(3)
    se3_recovered = xyzortho6d_to_se3(xyzortho6d)

    # check if the recovered SE(3) matrix is close to the original
    assert np.allclose(se3, se3_recovered), "Failed to recover SE(3) matrix from continuous 6D representation"


def ortho6d_R_conversion():
    # random continuous 6D representation
    ortho6d = np.random.rand(6)

    # convert to SO(3)
    rot = ortho6d_to_R(ortho6d)

    # convert back to continuous 6D representation
    ortho6d_recovered = R_to_ortho6d(rot)

    # check if the recovered continuous 6D representation is close to the original
    assert np.allclose(ortho6d, ortho6d_recovered), "Failed to recover continuous 6D representation from SO(3)"
