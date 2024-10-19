import threading
from abc import ABC, abstractmethod
import multiprocessing as mp
from multiprocessing import shared_memory
from typing import Literal

import cv2
import numpy as np


class CameraBase(ABC):
    def __init__(
        self,
        index: int,
        fps: int,
        resolution: tuple[int, int],
        **kwargs,
    ):
        self.index = index
        self.is_recording = False
        self.fps = fps

        self.resolution = resolution
        self.shms = {}
        self.image_arrays = {}

    def with_display(
        self,
        mode: Literal["mono", "stereo"],
        resolution: tuple[int, int],
        crop_sizes: tuple[int, int, int, int],
    ):
        self.display_mode = mode
        self.display_crop_sizes = [s if s != 0 else None for s in crop_sizes]

        t, b, l, r = crop_sizes
        resolution_cropped = (
            resolution[0] - t - b,
            resolution[1] - l - r,
        )

        self.display_shape = resolution_cropped

        num_images = 2 if mode == "stereo" else 1
        display_img_shape = (resolution_cropped[0], num_images * resolution_cropped[1], 3)
        self.shms["display"] = shared_memory.SharedMemory(
            create=True,
            size=np.prod(display_img_shape) * np.uint8().itemsize,  # type: ignore
        )
        self.image_arrays["display"] = np.ndarray(
            shape=display_img_shape,
            dtype=np.uint8,
            buffer=self.shms["display"].buf,
        )
        self.display_lock = threading.Lock()
        self.obs_lock = threading.Lock()

        self._video_path = mp.Array("c", bytes(256))
        self._flag_recording = mp.Value("i", 0)
        self._flag_marker = mp.Value("b", False)
        self._timestamp = mp.Value("d", 0)
        self.stop_event = mp.Event()
        return self

    @property
    def shared_memory_names(self) -> dict[str, str]:
        if self.shms:
            return {name: shm.name for name, shm in self.shms.items()}
        else:
            raise ValueError("Shared memory not initialized.")

    @property
    def shared_memory_size(self) -> dict[str, int]:
        if self.shms:
            return {name: shm.size for name, shm in self.shms.items()}
        else:
            raise ValueError("Shared memory not initialized.")

    @property
    def timestamp(self) -> float:
        with self._timestamp.get_lock():
            return self._timestamp.value

    @timestamp.setter
    def timestamp(self, value: float):
        with self._timestamp.get_lock():
            self._timestamp.value = value

    @property
    def flag_marker(self) -> bool:
        with self._flag_marker.get_lock():
            return bool(self._flag_marker.value)

    @flag_marker.setter
    def flag_marker(self, value: bool):
        with self._flag_marker.get_lock():
            self._flag_marker.value = value

    @property
    def video_path(self) -> str:
        with self._video_path.get_lock():
            return self._video_path.value.decode()

    @video_path.setter
    def video_path(self, value: str):
        with self._video_path.get_lock():
            self._video_path.value = value.encode()

    @property
    def available_sources(self) -> list[str]:
        return list(self.sources.keys())
    

    # @property
    # def available_sources(self) -> list[str]:
    #     raise NotImplementedError
    
    def send_to_display(self, data: dict[str, np.ndarray], marker=False):
        if "display" not in self.shms:
            return
        t, b, l, r = self.display_crop_sizes
        side_by_side = np.hstack(
            (
                data["left"][t : None if b is None else -b, l : None if r is None else -r],
                data["right"][t : None if b is None else -b, r : None if l is None else -l],
            )
        )
        # if marker:
        #     side_by_side = cv2.cvtColor(side_by_side, cv2.COLOR_RGB2GRAY)
        #     side_by_side = cv2.cvtColor(side_by_side, cv2.COLOR_GRAY2RGB)
        if marker:
            # draw markers on left and right frames
            width = side_by_side.shape[1]
            hieght = side_by_side.shape[0]

            side_by_side = cv2.circle(side_by_side, (int(width // 2 * 0.5), int(hieght * 0.2)), 15, (255, 0, 0), -1)
            side_by_side = cv2.circle(side_by_side, (int(width // 2 * 1.5), int(hieght * 0.2)), 15, (255, 0, 0), -1)
        with self.display_lock:
            np.copyto(self.image_arrays["display"], side_by_side)

    # @property
    # def resolution(self) -> tuple[int, int]:
    #     """height, width"""
    #     return self.resolution_cropped

    # @property
    # def img_height(self) -> int:
    #     return self.img_shape[0]

    # @property
    # def img_width(self) -> int:
    #     return self.img_shape[1]

    def start_recording(self, output_path: str):
        self.video_path = output_path
        self.is_recording = True
        with self._flag_recording.get_lock():
            self._flag_recording.value = 1

    def stop_recording(self):
        with self._flag_recording.get_lock():
            self._flag_recording.value = -1
        self.is_recording = False

    @abstractmethod
    def grab(self, sources: list[str]) -> tuple[float, dict[str, np.ndarray]]:
        raise NotImplementedError

    def post_process(
        self, data_dict: dict[str, np.ndarray], shape: tuple[int, int], crop_sizes: tuple[int, int, int, int]
    ) -> dict[str, np.ndarray]:
        for source, data in data_dict.items():
            data_dict[source] = CameraBase._post_process(source, data, shape, crop_sizes)
        return data_dict

    @abstractmethod
    def close(self): ...

    @staticmethod
    def _post_process(
        source: str, data: np.ndarray, shape: tuple[int, int], crop_sizes: tuple[int, int, int, int]
    ) -> np.ndarray:
        # cropped_img_shape = (240, 320) hxw
        # crop_sizes = (0, 0, int((1280-960)/2), int((1280-960)/2)) # (h_top, h_bottom, w_left, w_right)
        shape = (shape[1], shape[0])  # (w, h)
        crop_h_top, crop_h_bottom, crop_w_left, crop_w_right = crop_sizes
        if source == "left" or source == "depth":
            data = data[crop_h_top:-crop_h_bottom, crop_w_left:-crop_w_right]
            data = cv2.resize(data, shape)
        elif source == "right":
            data = data[crop_h_top:-crop_h_bottom, crop_w_right:-crop_w_left]
            data = cv2.resize(data, shape)

        return data
