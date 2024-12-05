"""
This module is adapted from https://github.com/huggingface/lerobot/blob/main/lerobot/common/robot_devices/cameras/intelrealsense.py
License: Apache-2.0 License
"""

import logging
import math
import multiprocessing as mp
import os
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Literal

import numpy as np
import pyrealsense2 as rs

from teleoperation.camera.utils import DisplayCamera, RecordCamera, delete_if_exists
from teleoperation.utils import get_timestamp_utc

logger = logging.getLogger(__name__)


def find_cameras(raise_when_empty=True) -> list[dict]:
    """
    Find the names and the serial numbers of the Intel RealSense cameras
    connected to the computer.
    adapted from https://github.com/huggingface/lerobot
    """

    cameras = []
    for device in rs.context().query_devices():
        serial_number = int(device.get_info(rs.camera_info.serial_number))
        name = device.get_info(rs.camera_info.name)
        cameras.append(
            {
                "serial_number": serial_number,
                "name": name,
            }
        )

    if raise_when_empty and len(cameras) == 0:
        raise OSError(
            "Not a single camera was detected. Try re-plugging, or re-installing `librealsense` and its python wrapper `pyrealsense2`, or updating the firmware."
        )

    return cameras


@dataclass
class CameraRealsenseConfig:
    fps: int
    width: int
    height: int
    use_depth: bool = False
    save_processes: int = 2
    save_threads: int = 4
    save_queue_size: int = 120


@dataclass
class DisplayConfig:
    key: str
    mode: Literal["mono", "stereo"]
    resolution: tuple[int, int]
    crop_sizes: tuple[int, int, int, int]


class CameraRealsenseSingle:
    def __init__(
        self,
        key: str,
        serial_number: int,
        camera_config: CameraRealsenseConfig,
    ):
        self.key = key
        self.serial_number = serial_number
        self.config = camera_config

        self.fps = camera_config.fps
        self.width = camera_config.width
        self.height = camera_config.height

        self.stop_event = threading.Event()
        self.timestamp = None

        self.images = {"color": None, "depth": None}
        self._lock = threading.Lock()

    def run(self):
        self._make_camera()
        while not self.stop_event.is_set():
            start = time.monotonic()
            try:
                frames = self.camera.wait_for_frames(timeout_ms=5000)
            except Exception:
                logger.warning(f"TimeoutError for IntelRealSenseCamera({self.serial_number}).")
            with self._lock:
                self.timestamp = get_timestamp_utc().timestamp()

                try:
                    color_frame = frames.get_color_frame()
                    color_image_rgb = np.asanyarray(color_frame.get_data())
                    self.images["color"] = color_image_rgb

                    if self.config.use_depth:
                        depth_frame = frames.get_depth_frame()
                        depth_image = np.asanyarray(depth_frame.get_data())
                        self.images["depth"] = depth_image
                except Exception as e:
                    logger.debug(f"Error while reading IntelRealSenseCamera({self.serial_number}) images: {e}")
                    pass
            taken = time.monotonic() - start
            time.sleep(max(1 / self.fps - taken, 0))

    def start(self):
        self.stop_event.clear()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def read(self):
        with self._lock:
            return self.timestamp, self.images

    def close(self):
        self.stop_event.set()
        self.camera.stop()

    def _make_camera(self):
        config = rs.config()
        config.enable_device(str(self.serial_number))

        if self.fps and self.width and self.height:
            # TODO(rcadene): can we set rgb8 directly?
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        else:
            config.enable_stream(rs.stream.color)

        if self.config.use_depth:
            if self.fps and self.width and self.height:
                config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            else:
                config.enable_stream(rs.stream.depth)

        self.camera = rs.pipeline()
        try:
            profile = self.camera.start(config)
            is_camera_open = True
        except RuntimeError:
            is_camera_open = False
            traceback.print_exc()

        if not is_camera_open or not profile:
            # Verify that the provided `serial_number` is valid before printing the traceback
            camera_infos = find_cameras()
            serial_numbers = [cam["serial_number"] for cam in camera_infos]
            if self.serial_number not in serial_numbers:
                raise ValueError(
                    f"`serial_number` is expected to be one of these available cameras {serial_numbers}, but {self.serial_number} is provided instead. "
                    "To find the serial number you should use, run `python lerobot/common/robot_devices/cameras/intelrealsense.py`."
                )

            raise OSError(f"Can't access IntelRealSenseCamera({self.serial_number}).")

        color_stream = profile.get_stream(rs.stream.color)
        color_profile = color_stream.as_video_stream_profile()
        actual_fps = color_profile.fps()
        actual_width = color_profile.width()
        actual_height = color_profile.height()

        # Using `math.isclose` since actual fps can be a float (e.g. 29.9 instead of 30)
        if self.fps is not None and not math.isclose(self.fps, actual_fps, rel_tol=1e-3):
            # Using `OSError` since it's a broad that encompasses issues related to device communication
            raise OSError(
                f"Can't set {self.fps=} for IntelRealSenseCamera({self.serial_number}). Actual value is {actual_fps}."
            )
        if self.width is not None and self.width != actual_width:
            raise OSError(
                f"Can't set {self.width=} for IntelRealSenseCamera({self.serial_number}). Actual value is {actual_width}."
            )
        if self.height is not None and self.height != actual_height:
            raise OSError(
                f"Can't set {self.height=} for IntelRealSenseCamera({self.serial_number}). Actual value is {actual_height}."
            )

        self.fps = round(actual_fps)
        self.width = round(actual_width)
        self.height = round(actual_height)


class CameraRealsenseMulti:
    def __init__(self, keys: dict[str, int], camera_config: CameraRealsenseConfig, display_config: DisplayConfig):
        self.cameras = {}
        for key, serial_number in keys.items():
            logger.info(f"Creating camera {key} with serial number {serial_number}")
            self.cameras[key] = CameraRealsenseSingle(key, serial_number, camera_config)

        self.fps = camera_config.fps
        self.display_config = display_config
        self.recorder = RecordCamera(
            camera_config.save_processes, camera_config.save_threads, camera_config.save_queue_size
        )
        self.display = DisplayCamera(
            self.display_config.mode, self.display_config.resolution, self.display_config.crop_sizes
        )

        self.stop_event = threading.Event()
        self.record_event = threading.Event()
        self._video_path = mp.Array("c", bytes(256))

        self.frame_id = 0
        self.timestamp = 0

    def start_recording(self, output_path: str):
        self.frame_id = 0
        self.video_path = output_path
        delete_if_exists(self.video_path)
        self.record_event.set()

    def stop_recording(self):
        self.record_event.clear()
        self.frame_id = 0

    @property
    def is_recording(self):
        return self.record_event.is_set()

    @property
    def video_path(self) -> str:
        with self._video_path.get_lock():
            return self._video_path.value.decode()

    @video_path.setter
    def video_path(self, value: str):
        with self._video_path.get_lock():
            self._video_path.value = value.encode()

    def run(self):
        while not self.stop_event.is_set():
            # while True:
            #     for key, cam in self.cameras.items():
            #         logger.warning(f"No image for camera {key}.")
            #         timestamp, images = cam.read()
            #         if images["color"] is not None:
            #             break
            #         else:
            #             time.sleep(0.1)
            skip = False
            for key, cam in self.cameras.items():
                timestamp, images = cam.read()

                if images["color"] is None:
                    logger.warning(f"No image for camera {key}.")
                    skip = True
                    continue

                if cam.config.use_depth and images["depth"] is None:
                    logger.warning(f"No depth image for camera {key}.")
                    skip = True
                    continue

                # display camera with the right key
                if self.display_config.key == key:
                    # print(f"Displaying frame for camera {key}")
                    self.display.put({"rgb": images["color"].copy()}, marker=self.is_recording)

                if self.is_recording:
                    # print(f"Recording frame {self.frame_id} for camera {key}")
                    self.recorder.put(images, self.frame_id, os.path.join(self.video_path, key), timestamp)
            if not skip:
                self.frame_id += 1
            time.sleep(1 / self.fps)

    def start(self):
        for camera in self.cameras.values():
            camera.start()
        self.recorder.start()
        self.stop_event.clear()
        self.record_event.clear()

        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

        return self

    def close(self):
        self.stop_event.set()
        for camera in self.cameras.values():
            camera.close()

        self.recorder.stop()


if __name__ == "__main__":
    cameras = find_cameras()
    print(cameras)
    if len(cameras) == 0:
        logger.error("No cameras detected.")
        exit(1)

    # # cameras = {"top": 239722071918, "right_wrist": 419122271585}
    # cameras = {"top": 239722071918, "left_wrist": 419122271348, "right_wrist": 419122271585, "bottom": 233522075865}

    # camera_config = CameraRealsenseConfig(fps=30, width=640, height=480, use_depth=True)
    # display_config = DisplayConfig(
    #     "top",
    #     "mono",
    #     (480, 640),
    #     (0, 0, 0, 0),
    # )

    # camera_multi = CameraRealsenseMulti(cameras, camera_config, display_config).start()
    # camera_multi.start_recording(os.path.join(DATA_DIR, "test_rec"))
    # print(camera_multi)
    # while True:
    #     # print(camera_multi.is_recording)
    #     time.sleep(1)
