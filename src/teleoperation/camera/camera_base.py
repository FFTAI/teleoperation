import multiprocessing as mp
import threading
from multiprocessing import shared_memory
from typing import Literal

import cv2
import numpy as np

from teleoperation.camera.utils import save_images_threaded


class RecordCamera:
    def __init__(self, num_processes: int = 1, num_threads: int = 4, queue_size: int = 30):
        super().__init__()
        self.num_processes = num_processes
        self.num_threads = num_threads
        self.save_queue = mp.Queue(maxsize=queue_size)
        self.processes = []

    def put(self, frames: dict[str, np.ndarray], frame_id: int, video_path: str, timestamp: float):
        for key, frame in frames.items():
            self.save_queue.put((frame, key, frame_id, video_path, timestamp))

    def start(self):
        for _ in range(self.num_processes):
            p = mp.Process(target=save_images_threaded, args=(self.save_queue, self.num_threads), daemon=True)
            p.start()
            self.processes.append(p)

    def stop(self):
        if self.save_queue is not None:
            self.save_queue.put(None)

        for p in self.processes:
            p.join()


class DisplayCamera:
    def __init__(
        self,
        mode: Literal["mono", "stereo"],
        resolution: tuple[int, int],
        crop_sizes: tuple[int, int, int, int],
    ):
        self.mode = mode
        self.crop_sizes = [s if s != 0 else None for s in crop_sizes]

        t, b, l, r = crop_sizes
        resolution_cropped = (
            resolution[0] - t - b,
            resolution[1] - l - r,
        )

        self.shape = resolution_cropped

        num_images = 2 if mode == "stereo" else 1

        display_img_shape = (resolution_cropped[0], num_images * resolution_cropped[1], 3)
        self.shm = shared_memory.SharedMemory(
            create=True,
            size=np.prod(display_img_shape) * np.uint8().itemsize,  # type: ignore
        )
        self.image_array = np.ndarray(
            shape=display_img_shape,
            dtype=np.uint8,
            buffer=self.shm.buf,
        )
        self.lock = threading.Lock()

        self._video_path = mp.Array("c", bytes(256))
        self._flag_marker = False

    @property
    def shm_name(self) -> str:
        return self.shm.name

    @property
    def shm_size(self) -> int:
        return self.shm.size

    def put(self, data: dict[str, np.ndarray], marker=False):
        t, b, l, r = self.crop_sizes

        if self.mode == "mono":
            if "rgb" in data:
                display_img = data["rgb"][t : None if b is None else -b, l : None if r is None else -r]
            elif "left" in data:
                display_img = data["left"][t : None if b is None else -b, l : None if r is None else -r]
            else:
                raise ValueError("Invalid data.")
        elif self.mode == "stereo":
            display_img = np.hstack(
                (
                    data["left"][t : None if b is None else -b, l : None if r is None else -r],
                    data["right"][t : None if b is None else -b, r : None if l is None else -l],
                )
            )
        else:
            raise ValueError("Invalid mode.")

        if marker:
            # draw markers on left and right frames
            width = display_img.shape[1]
            hieght = display_img.shape[0]

            if self.mode == "mono":
                display_img = cv2.circle(display_img, (int(width // 2), int(hieght * 0.2)), 15, (255, 0, 0), -1)
            elif self.mode == "stereo":
                display_img = cv2.circle(display_img, (int(width // 2 * 0.5), int(hieght * 0.2)), 15, (255, 0, 0), -1)
                display_img = cv2.circle(display_img, (int(width // 2 * 1.5), int(hieght * 0.2)), 15, (255, 0, 0), -1)
        with self.lock:
            np.copyto(self.image_array, display_img)
