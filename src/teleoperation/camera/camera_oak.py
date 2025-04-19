import logging
import multiprocessing as mp
import os
import queue
import threading
import time
from collections import defaultdict
from typing import Literal

import cv2
import depthai as dai
from depthai_sdk import OakCamera
from depthai_sdk.classes.packets import FramePacket

from teleoperation.camera.oak_utils import find_cameras
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
        eval_mode: bool = False,
    ):
        cameras = find_cameras(raise_when_empty=True)

        if len(cameras) > 1:
            logger.warning("Multiple cameras detected. Using the first one.")
        self.cam_info = list(cameras.values())[0]
        self.cam_info.name = key
        self.cam_info.fps = fps

        self.key = key
        self.fps = fps
        self.use_depth = use_depth
        self.eval_mode = eval_mode
        self.stereo_resolution = stereo_resolution
        self.color_resolution = color_resolution

        self.display = DisplayCamera(display_mode, display_resolution, display_crop_sizes)
        if self.eval_mode:
            self.recorder = None
        else:
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

        self.current_frames = defaultdict(lambda: None)

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

        self.cam_info.save_json(self.video_path)

        self.is_recording.set()

    def stop_recording(self):
        self.is_recording.clear()
        self.frame_id = 0

    def run(self):
        oak, q_display, q_obs = self._make_camera()
        ts_offset = None
        while not self.stop_event.is_set():
            self.oak.start()
            while self.oak.running() and self.q_obs is not None:
                start = time.monotonic()
                self.oak.poll()
                self.timestamp = get_timestamp_utc().timestamp()

                if self.q_display is not None and not self.eval_mode:
                    try:
                        p: FramePacket = self.q_display.get_queue().get(block=False)

                        left_frame = cv2.cvtColor(p[self.sources["left"]].frame, cv2.COLOR_GRAY2RGB)
                        right_frame = cv2.cvtColor(p[self.sources["right"]].frame, cv2.COLOR_GRAY2RGB)
                        self.display.put({"left": left_frame, "right": right_frame}, marker=self.is_recording.is_set())
                    except queue.Empty:
                        pass
                    except Exception as e:
                        logger.exception(e)

                try:
                    p_obs: FramePacket = self.q_obs.get_queue().get(block=False)
                    if self.is_recording.is_set() or self.eval_mode:
                        # logger.info(f"FPS: {self.q_obs.get_fps()}")
                        if self.use_depth:
                            # device_ts = dai.Clock.now()
                            # if ts_offset is None:
                            #     ts_offset = get_timestamp_utc().timestamp() - device_ts.total_seconds()
                            # logger.info(
                            #     f"device time diff: {device_ts.total_seconds() +  ts_offset - get_timestamp_utc().timestamp()}"
                            # )

                            # latencyMs = (
                            #     device_ts - p_obs[self.sources["rgb"]].msg.getTimestamp()
                            # ).total_seconds() * 1000
                            # logger.info(f"latency: {latencyMs}")
                            rgb_frame = cv2.cvtColor(p_obs[self.sources["rgb"]].frame, cv2.COLOR_BGR2RGB)
                            depth_frame = p_obs[self.sources["depth"]].frame
                        else:
                            rgb_frame = cv2.cvtColor(p_obs.frame, cv2.COLOR_BGR2RGB)
                            depth_frame = None

                        if not self.eval_mode and self.recorder is not None:
                            self.recorder.put(
                                {"rgb": rgb_frame, "depth": depth_frame},
                                self.frame_id,
                                self.video_path,
                                timestamp=self.timestamp,
                            )
                        elif self.eval_mode:
                            self.current_frames["rgb"] = rgb_frame
                            self.current_frames["depth"] = depth_frame
                        self.frame_id += 1

                except queue.Empty:
                    # logger.info("QUEUE EMPTY")
                    pass
                except Exception as e:
                    logger.exception(e)

                taken = time.monotonic() - start
                time.sleep(max(1 / self.fps - taken, 0))

    def grab(self):
        return {self.key: self.current_frames}

    def start(self):
        self.stop_event.clear()

        self.processes = []
        self.processes.append(threading.Thread(target=self.run, daemon=True))
        if not self.eval_mode and self.recorder is not None:
            self.recorder.start()
        for p in self.processes:
            p.start()
        return self

    def _make_camera(self):
        if self.oak is not None:
            self.oak.close()
        oak = OakCamera(device=self.cam_info.serial_number, args={"xlinkChunkSize": 0})
        stereo_fps = self.fps
        color_fps = self.fps

        if not self.eval_mode:
            left = oak.create_camera("left", resolution=self.stereo_resolution, fps=stereo_fps)
            right = oak.create_camera("right", resolution=self.stereo_resolution, fps=stereo_fps)
            q_display = oak.queue([left, right], max_size=3).configure_syncing(
                enable_sync=True, threshold_ms=int((1000 / stereo_fps) / 2)
            )
        else:
            left = None
            right = None
            q_display = None

        color = oak.create_camera("CAM_A", resolution=self.color_resolution, fps=color_fps)
        if self.color_resolution == "1080p":
            color.config_color_camera(isp_scale=(2, 3))

        if self.use_depth:
            stereo = oak.create_stereo(left=left, right=right, resolution=self.stereo_resolution, fps=stereo_fps)
            stereo.config_stereo(align=color, subpixel=False, lr_check=True)
            # stereo.node.setOutputSize(640, 360) # 720p, downscaled to 640x360 (decimation filter, median filtering)
            # On-device post processing for stereo depth
            config = stereo.node.initialConfig.get()
            stereo.node.setPostProcessingHardwareResources(3, 3)
            config.postProcessing.speckleFilter.enable = False
            config.postProcessing.thresholdFilter.minRange = 200
            config.postProcessing.thresholdFilter.maxRange = 3_000  # 3m
            config.postProcessing.decimationFilter.decimationFactor = 2
            config.postProcessing.decimationFilter.decimationMode = (
                dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN
            )
            stereo.node.initialConfig.set(config)

            logger.info(
                f"OAK camera: depth_unit: {stereo.node.initialConfig.getDepthUnit()}; max_disp: {stereo.node.initialConfig.getMaxDisparity()}"
            )

            q_obs = oak.queue([color, stereo], max_size=5).configure_syncing(
                threshold_ms=int((1000 / max(stereo_fps, color_fps)) / 2)
            )
        else:
            stereo = None
            q_obs = oak.queue([color], max_size=5)

        self.sources = {
            "rgb": color,
            "depth": stereo,
            "left": left,
            "right": right,
        }
        self.oak = oak
        self.q_display = q_display
        self.q_obs = q_obs
        return oak, q_display, q_obs

    def close(self):
        self.stop_event.set()
        self.recorder.stop()
        if self.oak is not None:
            self.oak.close()
        if self.processes is not None:
            for p in self.processes.reverse():
                p.join()
