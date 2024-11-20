from collections.abc import Sequence
from typing import Protocol, runtime_checkable

import sys
# sys.path.append("/data/teleoperation_dds/src/teleoperation/adapter/hands")
from pathlib import Path
script_dir = Path(__file__).resolve().parent  #teleoperation_dds/src/teleoperation/adapter/hands
sys.path.append(str(script_dir))
from .fourier_dexhand import FourierDexHand_12Dof
@runtime_checkable
class HandAdapter(Protocol):
    def init(self): ...
    def get_positions(self) -> Sequence[int]: ...
    def set_positions(self, positions: Sequence[int]): ...
    def reset(self): ...


class DummyDexHand:
    def __init__(self, dim=6):
        self.dim = dim

    def init(self): ...
    def get_positions(self) -> Sequence[int]:
        return [0] * self.dim

    def set_positions(self, positions: Sequence[int]): ...
    def reset(self): ...
