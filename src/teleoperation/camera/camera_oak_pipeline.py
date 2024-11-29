import logging
import multiprocessing as mp
import os
import threading
import time
from collections import defaultdict
from datetime import timedelta
from typing import Literal

import cv2
import depthai as dai
import numpy as np

from teleoperation.camera.utils import DisplayCamera, RecordCamera, delete_if_exists
from teleoperation.utils import get_timestamp_utc

logger = logging.getLogger(__name__)


class CameraOak:
    def __init__(
        self,
        key: str,
        fps: int,
        use_depth: bool,
        stereo_resolution: str,
        color_resolution: str,
        save_processes: int,
        save_threads: int,
        save_queue_size: int,
        display_mode: Literal["mono", "stereo"],
        display_resolution: tuple[int, int],
        display_crop_sizes: tuple[int, int, int, int],
    ):
        self.key = key
        self.fps = fps
        self.use_depth = use_depth
        self.stereo_resolution = stereo_resolution
        self.color_resolution = color_resolution

        self.display = DisplayCamera(display_mode, display_resolution, display_crop_sizes)
        self.recorder = RecordCamera(save_processes, save_threads, save_queue_size)
        self.stop_event = mp.Event()

        self.oak = None
        self.q_display = None
        self.q_obs = None
        self.sources = {}

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

    def run(self):
        pipeline = self._make_pipeline()
        ts_offset = None
        with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS) as device:
            q_rgbd = device.getOutputQueue("rgbd", 5, False)
            q_stereo = device.getOutputQueue("stereo", 10, False)
            diffs = defaultdict(lambda: np.array([]))
            while not self.stop_event.is_set():
                start = time.monotonic()
                self.timestamp = get_timestamp_utc().timestamp()

                try:
                    synced_msgs = q_stereo.tryGet()

                    if synced_msgs["left"] is not None and synced_msgs["right"] is not None:
                        left_frame = cv2.cvtColor(synced_msgs["left"].getCvFrame(), cv2.COLOR_GRAY2RGB)
                        right_frame = cv2.cvtColor(synced_msgs["right"].getCvFrame(), cv2.COLOR_GRAY2RGB)
                        self.display.put({"left": left_frame, "right": right_frame}, marker=self.is_recording.is_set())
                except Exception as e:
                    logger.warning(f"Stereo get exception: {e}")

                try:
                    synced_msgs = q_rgbd.tryGet()
                    if synced_msgs is not None:
                        output_frames = {"rgb": None, "depth": None}
                        for name, msg in synced_msgs:
                            if msg is None:
                                continue
                            frame = msg.getCvFrame()

                            latencyMs = (dai.Clock.now() - msg.getTimestamp()).total_seconds() * 1000
                            diffs[name] = np.append(diffs[name], latencyMs)
                            logger.info(
                                f"[{name}] Latency: {latencyMs:.2f} ms, Average latency: {np.average(diffs[name]):.2f} ms, Std: {np.std(diffs[name]):.2f}"
                            )

                            if self.is_recording.is_set():
                                if name == "video":
                                    output_frames["rgb"] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                if name == "disparity" and self.use_depth:
                                    # disparityMultiplier = 255.0 / device.getOutputQueue("xout").get().getFrame().getMax()
                                    # frame = (frame * disparityMultiplier).astype(np.uint8)
                                    output_frames["depth"] = frame
                        if self.is_recording.is_set():
                            self.recorder.put(
                                output_frames,
                                self.frame_id,
                                self.video_path,
                                timestamp=self.timestamp,
                            )
                            self.frame_id += 1
                except Exception as e:
                    logger.warning(f"Color get exception: {e}")

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

    def _make_pipeline(self):
        pipeline = dai.Pipeline()

        if self.stereo_resolution == "800p":
            stereo_resolution = dai.MonoCameraProperties.SensorResolution.THE_800_P
        elif self.stereo_resolution == "720p":
            stereo_resolution = dai.MonoCameraProperties.SensorResolution.THE_720_P
        elif self.stereo_resolution == "400p":
            stereo_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P
        else:
            logger.warning(f"Invalid stereo resolution: {self.stereo_resolution}, falling back to 400p")
            stereo_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P

        if self.color_resolution == "800p":
            color_resolution = dai.ColorCameraProperties.SensorResolution.THE_800_P
        elif self.color_resolution == "720p":
            color_resolution = dai.ColorCameraProperties.SensorResolution.THE_720_P
        elif self.color_resolution == "1080p":
            color_resolution = dai.ColorCameraProperties.SensorResolution.THE_1080_P
        else:
            logger.warning(f"Invalid color resolution: {self.color_resolution}, falling back to 800p")
            color_resolution = dai.ColorCameraProperties.SensorResolution.THE_800_P

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        color = pipeline.create(dai.node.ColorCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        sync = pipeline.create(dai.node.Sync)
        stereo_sync = pipeline.create(dai.node.Sync)

        rgbd_out = pipeline.create(dai.node.XLinkOut)
        rgbd_out.setStreamName("rgbd")

        stereo_out = pipeline.create(dai.node.XLinkOut)
        stereo_out.setStreamName("stereo")

        mono_left.setResolution(stereo_resolution)
        mono_left.setCamera("left")
        mono_left.setFps(self.fps)
        mono_right.setResolution(stereo_resolution)
        mono_right.setCamera("right")
        mono_right.setFps(self.fps)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)

        config = stereo.initialConfig.get()
        config.postProcessing.speckleFilter.enable = True
        config.postProcessing.speckleFilter.speckleRange = 50
        config.postProcessing.thresholdFilter.minRange = 200  # 0.2m
        config.postProcessing.thresholdFilter.maxRange = 3_000  # 3m
        stereo.initialConfig.set(config)
        stereo.setPostProcessingHardwareResources(3, 3)

        color.setCamera("color")
        color.setResolution(color_resolution)
        color.setFps(self.fps)

        sync.setSyncThreshold(timedelta(milliseconds=int(1000 / self.fps / 2)))
        stereo_sync.setSyncThreshold(timedelta(milliseconds=int(1000 / self.fps / 2)))

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        stereo.disparity.link(sync.inputs["depth"])
        color.video.link(sync.inputs["rgb"])
        sync.out.link(rgbd_out.input)

        mono_left.out.link(stereo_sync.inputs["left"])
        mono_right.out.link(stereo_sync.inputs["right"])
        stereo_sync.out.link(stereo_out.input)

        return pipeline

    def close(self):
        self.stop_event.set()
        self.recorder.stop()
        if self.oak is not None:
            self.oak.close()
        if self.processes is not None:
            for p in self.processes.reverse():
                p.join()
