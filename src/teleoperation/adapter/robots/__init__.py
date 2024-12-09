from collections.abc import Sequence
from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class RobotAdapter(Protocol):
    @property
    def joint_positions(self) -> np.ndarray: ...

    def connect(self): ...

    def disconnect(self): ...

    def command_joints(self, joint_positions, gravity_compensation=False): ...

    def init_command_joints(self, positions): ...

    def stop_joints(self): ...

    def observe(self) -> tuple: ...


class DummyRobot:
    def __init__(self, dim=32):
        self.dim = dim

    @property
    def joint_positions(self):
        return np.zeros(self.dim)

    def connect(self): ...

    def disconnect(self): ...

    def command_joints(self, joint_positions, gravity_compensation=False): ...

    def init_command_joints(self, positions): ...

    def stop_joints(self): ...

    def observe(self) -> tuple:
        return (np.zeros(self.dim),)
