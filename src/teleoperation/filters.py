import math

import numpy as np
from numba import jit
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


@jit
def smoothing_factor(t_e, cutoff):
    r = 2 * math.pi * cutoff * t_e
    return r / (r + 1)


@jit
def exponential_smoothing(a, x, x_prev):
    return a * x + (1 - a) * x_prev


def rotational_exponential_smoothing(a, x, x_prev):
    s = Slerp([0, 1], R.from_quat([x_prev, x]))
    x_hat = s(a)
    return x_hat.as_quat()


class OneEuroFilter:
    def __init__(
        self,
        min_cutoff=1.0,
        beta=0.0,
        d_cutoff=1.0,
    ):
        """Initialize the one euro filter for a 14-dimensional numpy array."""
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)

        self.data_shape = None

        self.t_prev = None
        self.x_prev = None
        self.dx_prev = None

        self.smoothing_fn = exponential_smoothing

    def next(self, t, x, dx0=None):
        """Compute the filtered signal for a 14-dimensional numpy array."""
        if self.t_prev is None:
            self.data_shape = x.shape
            self.t_prev = float(t)
            self.x_prev = np.array(x, dtype=float)
            if dx0 is None:
                self.dx_prev = np.zeros_like(x)
            else:
                self.dx_prev = np.array(dx0, dtype=float)
            return x

        if x.shape != self.data_shape:
            raise ValueError("Unexpected data shape")

        t_e = t - self.t_prev

        # The filtered derivative of the signal
        a_d = smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.smoothing_fn(a_d, dx, self.dx_prev)

        # The filtered signal
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = smoothing_factor(t_e, cutoff)
        x_hat = self.smoothing_fn(a, x, self.x_prev)

        # Memorize the previous values
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat


class LPRotationFilter:
    """https://github.com/Dingry/bunny_teleop_server/blob/main/bunny_teleop_server/utils/robot_utils.py"""

    def __init__(self, alpha):
        self.alpha = alpha
        self.is_init = False

        self.y = None

    def next(self, x: np.ndarray):
        assert x.shape == (4,)

        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()

        self.y = rotational_exponential_smoothing(self.alpha, x, self.y)

        return self.y.copy()

    def next_mat(self, x: np.ndarray):
        """take and return rotation matrix instead of quat"""
        assert x.shape == (3, 3) or x.shape == (4, 4)

        if x.shape == (4, 4):
            x = x[:3, :3]

        x = R.from_matrix(x).as_quat()
        next_x_quat = self.next(x)

        return R.from_quat(next_x_quat).as_matrix()

    def reset(self):
        self.y = None
        self.is_init = False
