import logging
import multiprocessing as mp
import threading
import time
from typing import Literal

import cv2
import numpy as np
import pyzed.sl as sl

from teleoperation.camera.utils import DisplayCamera, delete_if_exists

logger = logging.getLogger(__name__)

MAX_DISTANCE_MM = 800
MIN_DISTANCE_MM = 150
MAX_DISTANCE = MAX_DISTANCE_MM / 1000
MIN_DISTANCE = MIN_DISTANCE_MM / 1000


class CameraZed:
    def __init__(
        self,
        key: str,
        fps: int,
        display_mode: Literal["mono", "stereo"],
        display_resolution: tuple[int, int],
        display_crop_sizes: tuple[int, int, int, int],
    ):
        self.key = key
        self.fps = fps
        self.display = DisplayCamera(display_mode, display_resolution, display_crop_sizes)
        # self.recorder = RecordCamera()
        self.stop_event = mp.Event()

        self.zed, self.sources, self.runtime_parameters = self._make_camera()

        self.episode_id = 0
        self.frame_id = 0
        self.is_recording = False
        self._video_path = mp.Array("c", bytes(256))
        self._flag_recording = mp.Value("i", 0)
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
        delete_if_exists(self.video_path)
        self.is_recording = True
        with self._flag_recording.get_lock():
            self._flag_recording.value = 1

    def stop_recording(self):
        self.frame_id = 0
        self.is_recording = False
        with self._flag_recording.get_lock():
            self._flag_recording.value = -1

    def run(self):
        while not self.stop_event.is_set() and self.zed is not None:
            start = time.monotonic()
            with self._flag_recording.get_lock():
                if self._flag_recording.value == 1:
                    output_path = self.video_path + f"_{self.key}"
                    recording_params = sl.RecordingParameters(output_path, sl.SVO_COMPRESSION_MODE.H264)
                    err = self.zed.enable_recording(recording_params)
                    if err != sl.ERROR_CODE.SUCCESS:
                        logger.error(f"Failed to start recording: {err}")
                    self._flag_recording.value = 0
                elif self._flag_recording.value == -1:
                    self.zed.disable_recording()
                    self._flag_recording.value = 0

            timestamp, images_dict = self.grab(sources=["left", "right"])
            self.timestamp = timestamp

            self.display.put(images_dict, marker=self.is_recording)

            taken = time.monotonic() - start
            time.sleep(max(1 / self.fps - taken, 0))

    def grab(self, sources: list[str]) -> tuple[float, dict[str, np.ndarray]]:
        err = self.zed.grab(self.runtime_parameters)

        if err == sl.ERROR_CODE.SUCCESS:
            if "left" in sources or "side_by_side" in sources:
                self.zed.retrieve_image(self.sources["left"], sl.VIEW.LEFT)
            if "right" in sources or "side_by_side" in sources:
                self.zed.retrieve_image(self.sources["right"], sl.VIEW.RIGHT)
            if "depth" in sources:
                self.zed.retrieve_measure(self.sources["depth"], sl.MEASURE.DEPTH)
            if "point_cloud" in sources:
                self.zed.retrieve_measure(self.sources["point_cloud"], sl.MEASURE.XYZRGBA)
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        else:
            raise Exception(f"Failed to grab image: {err}")

        out = {}

        if "depth" in sources:
            depth_data = self.sources["depth"].get_data()
            depth_data = np.clip(depth_data, MIN_DISTANCE_MM, MAX_DISTANCE_MM)
            depth_data = (depth_data - MIN_DISTANCE_MM) / (MAX_DISTANCE_MM - MIN_DISTANCE_MM)
            # depth_image_normalized = (depth_image_normalized * 255).astype(np.uint8)
            out["depth"] = depth_data

        for source in sources:
            if source == "side_by_side":
                continue
            if source == "left" or source == "right":
                out[source] = cv2.cvtColor(
                    self.sources[source].get_data(),
                    cv2.COLOR_BGRA2RGB,
                )
            else:
                out[source] = self.sources[source].get_data()

        return timestamp.get_milliseconds(), out

    def start(self):
        self.stop_event.clear()

        self.processes = []
        self.processes.append(threading.Thread(target=self.run, daemon=True))
        # self.processes.append(mp.Process(target=save_images_threaded, args=(self.save_queue, 4), daemon=True))
        for p in self.processes:
            p.start()
        return self

    def _make_camera(self):
        sources: dict[str, sl.Mat] = {
            "left": sl.Mat(),
            "right": sl.Mat(),
            "depth": sl.Mat(),
            "point_cloud": sl.Mat(),
        }
        runtime_parameters = sl.RuntimeParameters()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = (
            sl.RESOLUTION.HD720
        )  # Use HD720 opr HD1200 video mode, depending on camera type.
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # Use ULTRA depth mode
        init_params.coordinate_units = sl.UNIT.METER  # Use millimeter units (for depth measurements)
        init_params.depth_minimum_distance = MIN_DISTANCE
        init_params.depth_maximum_distance = MAX_DISTANCE
        init_params.camera_resolution.width = 720
        init_params.camera_resolution.height = 1280
        init_params.camera_fps = self.fps  # Set fps at 60

        zed = sl.Camera()
        err = zed.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(err) + ". Exit program.")
            exit()

        return zed, sources, runtime_parameters

    def close(self):
        self.stop_event.set()
        if self.zed is not None:
            self.zed.close()
        if self.processes is not None:
            for p in self.processes.reverse():
                p.join()
