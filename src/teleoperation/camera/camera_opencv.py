"""
Part of this module is adapted from https://github.com/huggingface/lerobot/blob/main/lerobot/common/robot_devices/cameras/opencv.py
License: Apache-2.0 License
"""

# The maximum opencv device index depends on your operating system. For instance,
# if you have 3 cameras, they should be associated to index 0, 1, and 2. This is the case
# on MacOS. However, on Ubuntu, the indices are different like 6, 16, 23.
# When you change the USB port or reboot the computer, the operating system might
# treat the same cameras as new devices. Thus we select a higher bound to search indices.
import logging
import multiprocessing as mp
import os
import platform
import threading
import time
from pathlib import Path
from typing import Literal

import cv2

from teleoperation.camera.utils import DisplayCamera, RecordCamera, delete_if_exists

logger = logging.getLogger(__name__)
cv2.setNumThreads(1)
MAX_OPENCV_INDEX = 60


def find_cameras(raise_when_empty=False, max_index_search_range=MAX_OPENCV_INDEX, mock=False) -> list[dict]:
    cameras = []
    if platform.system() == "Linux":
        logger.info(
            "Linux detected. Finding available camera indices through scanning '/dev/fourier_cam*' then '/dev/video*' ports"
        )
        possible_ports = [str(port) for port in Path("/dev").glob("fourier_cam*")]
        prefix = "/dev/fourier_cam"

        if len(possible_ports) == 0:
            logger.info("No cameras found in '/dev/fourier_cam*' ports. Trying '/dev/video*' ports")
            possible_ports = [str(port) for port in Path("/dev").glob("video*")]
            prefix = "/dev/video"

        ports = _find_cameras(possible_ports, mock=mock)
        for port in ports:
            cameras.append(
                {
                    "port": port,
                    "index": int(port.removeprefix(prefix)),
                }
            )
    else:
        logger.info(
            "Mac or Windows detected. Finding available camera indices through "
            f"scanning all indices from 0 to {MAX_OPENCV_INDEX}"
        )
        possible_indices = range(max_index_search_range)
        indices = _find_cameras(possible_indices, mock=mock)
        for index in indices:
            cameras.append(
                {
                    "port": None,
                    "index": index,
                }
            )

    return cameras


def _find_cameras(possible_camera_ids: list[int | str], raise_when_empty=False, mock=False) -> list[int | str]:
    import cv2

    camera_ids = []
    for camera_idx in possible_camera_ids:
        camera = cv2.VideoCapture(camera_idx)
        is_open = camera.isOpened()
        camera.release()

        if is_open:
            logger.info(f"Camera found at index {camera_idx}")
            camera_ids.append(camera_idx)

    if raise_when_empty and len(camera_ids) == 0:
        raise OSError(
            "Not a single camera was detected. Try re-plugging, or re-installing `opencv2`, "
            "or your camera driver, or make sure your camera is compatible with opencv2."
        )

    return camera_ids


class CameraOpencv:
    def __init__(
        self,
        key: str,
        port: str,
        fps: int,
        width: int,
        height: int,
        frame_type: Literal["normal", "side_by_side"],
        save_processes: int,
        save_threads: int,
        save_queue_size: int,
        display_mode: Literal["mono", "stereo"],
        display_resolution: tuple[int, int],
        display_crop_sizes: tuple[int, int, int, int],
    ):
        self.key = key
        self.fps = fps
        self.port = port
        self.width = width
        self.height = height
        self.frame_type = frame_type
        self.display = DisplayCamera(display_mode, display_resolution, display_crop_sizes)
        self.recorder = RecordCamera(save_processes, save_threads, save_queue_size)
        self.stop_event = mp.Event()

        self.episode_id = 0
        self.frame_id = 0
        self.is_recording = threading.Event()
        self._video_path = mp.Array("c", bytes(256))
        self._timestamp = 0

    @property
    def timestamp(self) -> float:
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value: float):
        self._timestamp = value

    @property
    def video_path(self) -> str:
        with self._video_path.get_lock():
            return self._video_path.value.decode()

    @video_path.setter
    def video_path(self, value: str):
        with self._video_path.get_lock():
            self._video_path.value = value.encode()

    def start_recording(self, output_path: str):
        self.frame_id = 0
        self.video_path = os.path.join(output_path, self.key)
        delete_if_exists(self.video_path)
        self.is_recording.set()

    def stop_recording(self):
        self.is_recording.clear()
        self.frame_id = 0

    def start(self):
        self.stop_event.clear()

        self.processes = []
        self.processes.append(threading.Thread(target=self.run, daemon=True))
        self.recorder.start()
        for p in self.processes:
            p.start()
        return self

    def _make_camera(self):
        camera = cv2.VideoCapture(self.port)

        if camera is None or not camera.isOpened():
            raise OSError(f"Cannot open camera at index {self.port}")

        num_images = 2 if self.frame_type == "side_by_side" else 1

        camera.set(cv2.CAP_PROP_FPS, self.fps)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width * num_images)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.fps = round(camera.get(cv2.CAP_PROP_FPS))
        # self.width = round(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = round(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

        logger.info(
            f"Camera {self.key} opened at port {self.port} with resolution {self.width * num_images}x{self.height} and fps {self.fps}"
        )

        return camera

    def run(self):
        self.cam = self._make_camera()
        while not self.stop_event.is_set():
            start = time.monotonic()
            ret, frame = self.cam.read()

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            self.timestamp = time.monotonic()
            if ret:
                if self.frame_type == "side_by_side":
                    left_frame = frame[:, : self.width, :]
                    right_frame = frame[:, self.width :, :]
                    self.display.put({"left": left_frame, "right": right_frame}, marker=self.is_recording.is_set())
                else:
                    self.display.put({"left": frame}, marker=self.is_recording.is_set())
                if self.is_recording.is_set():
                    left_frame = frame[:, : self.width, :]
                    right_frame = frame[:, self.width :, :]
                    self.recorder.put(
                        {"left": left_frame, "right": right_frame}, self.frame_id, self.video_path, self.timestamp
                    )
                    self.frame_id += 1

            taken = time.monotonic() - start
            time.sleep(max(1 / self.fps - taken, 0))


if __name__ == "__main__":
    cameras = find_cameras(raise_when_empty=True)
    print(cameras)

    import cv2

    cv2.setNumThreads(1)

    cam = cv2.VideoCapture(cameras[0]["index"])

    if cam is None or not cam.isOpened():
        raise OSError(f"Cannot open camera at index {cameras[0]['index']}")

    cv2.namedWindow("camera", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cam.read()
        if ret:
            cv2.imshow("camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(1 / 30)

    cv2.destroyAllWindows()
