import numpy as np
from pynput import keyboard


def mat_update(prev_mat, mat):
    if np.linalg.det(mat) == 0:
        return prev_mat
    else:
        return mat


def fast_mat_inv(mat):
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

    @property
    def space_pressed(self):
        if self._space_pressed:
            self._space_pressed = False
            return True
        return False

    def start(self):
        self.listener.start()

    def on_press(self, key):
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
