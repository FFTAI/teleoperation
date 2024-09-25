from abc import ABC, abstractmethod

import numpy as np


class CameraBase(ABC):
    def __init__(
        self,
        index: int,
        fps: int,
        resolution: tuple[int, int],
        crop_size_h: int,
        crop_size_w: int,
        open: bool,
        **kwargs,
    ):
        self.index = index
        self.is_recording = False

        self.crop_size_h = crop_size_h
        self.crop_size_w = crop_size_w
        resolution_cropped = (
            resolution[0] - crop_size_h,
            resolution[1] - 2 * crop_size_w,
        )
        self.img_shape = (resolution_cropped[0], 2 * resolution_cropped[1], 3)

    @property
    def resolution(self) -> tuple[int, int]:
        """height, width"""
        return self.img_shape[:2]

    @property
    def img_height(self) -> int:
        return self.img_shape[0]

    @property
    def img_width(self) -> int:
        return self.img_shape[1]

    @abstractmethod
    def start_recording(self, output_path: str):
        pass

    @abstractmethod
    def stop_recording(self):
        pass

    @abstractmethod
    def grab(self, gray: bool = False, enable_depth: bool = False) -> tuple[float, np.ndarray]:
        raise NotImplementedError

    @abstractmethod
    def close(self):
        pass
