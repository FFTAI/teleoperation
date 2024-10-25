import logging
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent
ASSET_DIR = PROJECT_ROOT.parent.parent / "assets"
CONFIG_DIR = PROJECT_ROOT.parent.parent / "configs"
DATA_DIR = PROJECT_ROOT.parent.parent / "data"
RECORD_DIR = DATA_DIR / "recordings"
LOG_DIR = DATA_DIR / "logs"
CERT_DIR = PROJECT_ROOT.parent.parent / "certs"

def format_episode_id(episode_id):
    return f"{episode_id:09d}"

def get_timestamp_utc():
    return datetime.now(timezone.utc)


def se3_to_xyzortho6d(se3):
    """
    Convert SE(3) to continuous 6D rotation representation.
    """
    so3 = se3[:3, :3]
    xyz = se3[:3, 3]
    ortho6d = so3_to_ortho6d(so3)
    return np.concatenate([xyz, ortho6d])

def so3_to_ortho6d(so3):
    """
    Convert to continuous 6D rotation representation adapted from
    On the Continuity of Rotation Representations in Neural Networks
    https://arxiv.org/pdf/1812.07035.pdf
    https://github.com/papagina/RotationContinuity/blob/master/sanity_test/code/tools.py
    """
    return so3[:, :2].transpose().reshape(-1)

def ortho6d_to_so3(ortho6d):
    """
    Convert from continuous 6D rotation representation to SO(3)
    """
    # TODO
    raise NotImplementedError

def se3_to_xyzquat(se3):
    se3 = np.asanyarray(se3).astype(float)
    if se3.shape != (4, 4):
        raise ValueError("Input must be a 4x4 matrix")
    return _se3_to_xyzquat(se3)


def xyzquat_to_se3(xyzquat):
    xyzquat = np.asanyarray(xyzquat).astype(float)
    if xyzquat.shape != (7,):
        raise ValueError("Input must be a 7-element array")
    return _xyzquat_to_se3(xyzquat)


def _se3_to_xyzquat(se3):
    translation = se3[:3, 3]
    rotmat = se3[:3, :3]

    quat = R.from_matrix(rotmat).as_quat()

    xyzquat = np.concatenate([translation, quat])
    return xyzquat


def _xyzquat_to_se3(xyzquat):
    translation = xyzquat[:3]
    quat = xyzquat[3:]

    rotmat = R.from_quat(quat).as_matrix()

    se3 = np.eye(4)
    se3[:3, :3] = rotmat
    se3[:3, 3] = translation

    return se3


def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat
    else:
        return mat


def fast_mat_inv(mat):
    mat = np.asarray(mat)
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret


def remap(x, old_min, old_max, new_min, new_max, clip=True):
    old_min = np.array(old_min)
    old_max = np.array(old_max)
    new_min = np.array(new_min)
    new_max = np.array(new_max)
    x = np.array(x)
    tmp = (x - old_min) / (old_max - old_min)
    if clip:
        tmp = np.clip(tmp, 0, 1)
    return new_min + tmp * (new_max - new_min)


class KeyboardListener:
    def __init__(self):
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self._space_pressed = False
        logger.debug("Keyboard listener initialized")

    @property
    def space_pressed(self):
        if self._space_pressed:
            self._space_pressed = False
            return True
        return False

    def start(self):
        self.listener.start()

    def on_press(self, key):
        logger.debug(f"Key pressed: {key}")
        try:
            if key == keyboard.Key.space:
                self._space_pressed = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key == keyboard.Key.space:
                self._space_pressed = False
        except AttributeError:
            pass

    def stop(self):
        self.listener.stop()
