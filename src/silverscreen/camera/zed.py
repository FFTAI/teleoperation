import multiprocessing as mp
import time

import cv2
import numpy as np
import pyzed.sl as sl
from loguru import logger

from .camera_base import CameraBase

# resolution = (720, 1280)
# crop_size_w = 1
# crop_size_h = 0
# resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)
MAX_DISTANCE_MM = 800
MIN_DISTANCE_MM = 150
MAX_DISTANCE = MAX_DISTANCE_MM / 1000
MIN_DISTANCE = MIN_DISTANCE_MM / 1000


class CamZed(CameraBase):
    def __init__(
        self,
        index: int = 0,
        fps: int = 60,
    ):
        super().__init__(index, fps, (720, 1280))

        self.sources: dict[str, sl.Mat] = {
            "left": sl.Mat(),
            "right": sl.Mat(),
            "depth": sl.Mat(),
            "point_cloud": sl.Mat(),
        }
        self.runtime_parameters = sl.RuntimeParameters()

    def start(self):
        self.process = mp.Process(target=self.run)
        self.process.daemon = True
        self.process.start()
        return self

    def run(self):
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

        self.zed = sl.Camera()
        err = self.zed.open(init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(err) + ". Exit program.")
            exit()

        while not self.stop_event.is_set():
            start = time.monotonic()
            with self._flag_recording.get_lock():
                if self._flag_recording.value == 1:
                    output_path = self.video_path
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

            self.send_to_display(images_dict, marker=self.flag_marker)

            taken = time.monotonic() - start
            time.sleep(max(1 / self.fps - taken, 0))

    def grab(self, sources: list[str]) -> tuple[float, dict[str, np.ndarray]]:
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
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
            raise Exception("Failed to grab image")

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

    def close(self):
        for source in self.sources.values():
            source.free(sl.MEM.CPU)
        self.zed.close()
