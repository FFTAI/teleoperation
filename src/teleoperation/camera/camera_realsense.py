import logging
import multiprocessing as mp
import threading
import time
from typing import Literal
from warnings import deprecated

import cv2
import numpy as np
import pyrealsense2 as rs

from teleoperation.camera.utils import DisplayCamera, RecordCamera
from teleoperation.utils import get_timestamp_utc

logger = logging.getLogger(__name__)


@deprecated(reason="This class is deprecated and will be removed in a future release. Please use camera_multi instead.")
class CameraRealsense:
    def __init__(
        self,
        index: int,
        fps: int,
        display_mode: Literal["mono", "stereo"],
        display_resolution: tuple[int, int],
        display_crop_sizes: tuple[int, int, int, int],
    ):
        self.index = index
        self.fps = fps
        self.display = DisplayCamera(display_mode, display_resolution, display_crop_sizes)
        self.recorder = RecordCamera()
        self.stop_event = mp.Event()

        self.camera = None

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
        self.video_path = output_path
        self.is_recording.set()

    def stop_recording(self):
        self.is_recording.clear()
        self.frame_id = 0

    def run(self):
        self.camera = self._make_camera()
        while not self.stop_event.is_set():
            while True:
                start = time.monotonic()
                frames = self.camera.wait_for_frames(timeout_ms=5000)
                self.timestamp = get_timestamp_utc().timestamp()
                try:
                    color_frame = frames.get_color_frame()
                    # Process frames for display
                    color_image = np.asanyarray(color_frame.get_data())
                    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

                    self.display.put(
                        {"left": color_image_rgb, "right": color_image_rgb}, marker=self.is_recording.is_set()
                    )

                    if self.is_recording.is_set():
                        depth_frame = frames.get_depth_frame()
                        depth_image = np.asanyarray(depth_frame.get_data())
                        self.recorder.put(
                            {"rgb": color_image_rgb, "depth": depth_image},
                            self.frame_id,
                            self.video_path,
                            self.timestamp,
                        )
                        self.frame_id += 1
                except:  # noqa: E722
                    pass

                taken = time.monotonic() - start
                time.sleep(max(1 / self.fps - taken, 0))

    def start(self):
        self.stop_event.clear()

        self.processes = []
        self.processes.append(threading.Thread(target=self.run, daemon=True))
        self.recorder.start()
        for p in self.processes:
            p.start()
        return self

    def _make_camera(self):
        if self.camera is not None:
            self.camera.stop()

        # Initialize RealSense pipeline and configuration
        config = rs.config()

        # Enable streams for color, infrared (left and right), and depth
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps)  # RGB
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)  # Depth

        # Start pipeline
        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        device = profile.get_device()
        color_sensor = device.first_color_sensor()
        color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        color_sensor.set_option(rs.option.enable_auto_white_balance, 0)
        color_sensor.set_option(rs.option.exposure, 150)
        color_sensor.set_option(rs.option.white_balance, 5900)

        # Configure post-processing settings for depth data
        depth_sensor = device.first_depth_sensor()
        depth_sensor.set_option(rs.option.enable_auto_exposure, True)
        depth_sensor.set_option(rs.option.visual_preset, rs.rs400_visual_preset.high_accuracy)

        return pipeline

    def close(self):
        self.stop_event.set()
        self.recorder.stop()

        # Stop RealSense pipeline
        if self.camera is not None:
            self.camera.stop()

        # Close and join any active processes
        if self.processes is not None:
            for p in self.processes.reverse():
                p.join()
