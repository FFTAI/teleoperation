import logging
import multiprocessing as mp
import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import cv2
import depthai as dai
from depthai_sdk import OakCamera

if TYPE_CHECKING:
    from depthai_sdk.classes.packet_handlers import QueuePacketHandler
    from depthai_sdk.classes.packets import FramePacket

from teleoperation.camera.oak_utils import find_cameras
from teleoperation.camera.utils import CameraInfo, DisplayCamera, RecordCamera, delete_if_exists
from teleoperation.utils import get_timestamp_utc

logger = logging.getLogger(__name__)


@dataclass
class CameraOakConfig:
    fps: int
    width: int
    height: int
    use_depth: bool = False
    resolution: str = "800p"


@dataclass
class DisplayConfig:
    key: str
    mode: Literal["mono", "stereo"]
    crop_sizes: tuple[int, int, int, int]
    resolution: str = "400p"


class CameraMultiOak:
    def __init__(
        self,
        keys: dict[str, str],
        camera_config: CameraOakConfig,
        display_config: DisplayConfig,
        save_processes: int,
        save_threads: int,
        save_queue_size: int,
        eval_mode: bool = False,
    ):
        self.fps = camera_config.fps

        cameras = find_cameras(raise_when_empty=True, type_str="oak-d-w-97")

        assert len(cameras) >= len(keys), f"Expected {len(keys)} cameras, found {len(cameras)}"

        if len(cameras) == 1 and len(keys) == 1:
            # If only one camera is found and one camera is requested, use the first one

            keys = {list(keys.keys())[0]: list(cameras.keys())[0]}
            logger.warning(f"Only one camera found and only one camera requested: {list(cameras.keys())[0]}")

        assert set(cameras.keys()).issuperset(
            set(keys.values())
        ), f"Expected cameras with serials {keys.values()}, found {cameras.keys()}"

        self.camera_infos: dict[str, CameraInfo] = {}
        for key, serial in keys.items():
            if serial not in cameras:
                raise ValueError(f"Camera with serial {serial} not found")
            logger.info(f"Camera {key} found with serial {serial}")

            self.camera_infos[key] = cameras[serial]

            self.camera_infos[key].name = key
            self.camera_infos[key].fps = camera_config.fps

            self.camera_infos[key].width = camera_config.width

        self.use_depth = camera_config.use_depth
        self.eval_mode = eval_mode

        self.display_config = display_config
        self.display = DisplayCamera(display_config.mode, display_config.resolution, display_config.crop_sizes)
        if self.eval_mode:
            self.recorder = None
        else:
            self.recorder = RecordCamera(save_processes, save_threads, save_queue_size)
        self.stop_event = mp.Event()

        self.cameras: dict[str, tuple[OakCamera, QueuePacketHandler | None, QueuePacketHandler]] = {}
        self.sources = {}

        self.camera_config = camera_config

        self.episode_id = 0
        self.frame_id = 0
        self.is_recording = threading.Event()
        self._video_path = mp.Array("c", bytes(256))
        self._timestamp = 0

        self.current_frames = {k: {"rgb": None, "depth": None} for k in self.camera_infos.keys()}

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

        video_path = Path(self.video_path)
        video_path.mkdir(parents=True, exist_ok=True)

        for key, val in self.camera_infos.items():
            camera_path = video_path / key
            camera_path.mkdir(parents=True, exist_ok=True)
            val.save_json(camera_path)

        self.is_recording.set()

    def stop_recording(self):
        self.is_recording.clear()
        self.frame_id = 0

    def run(self):
        for key, info in self.camera_infos.items():
            self.cameras[key] = self._make_camera(key, info.serial_number)

        ts_offset = None
        while not self.stop_event.is_set():
            for oak, _, _ in self.cameras.values():
                oak.start()

            oaks = [oak for oak, _, _ in self.cameras.values()]
            # q_obss = [q_obs for _, _, q_obs in self.cameras.values()]
            q_displays = [q_display for _, q_display, _ in self.cameras.values()]

            # only 1 camera can be displayed at a time
            assert (
                len([q_display for q_display in q_displays if q_display is not None]) <= 1
            ), f"More than one display camera found: {q_displays}"
            q_display = next((q_display for q_display in q_displays if q_display is not None), None)
            while all((oak.running() and q_obs is not None) for oak, _, q_obs in self.cameras.values()):
                start = time.monotonic()
                for oak in oaks:
                    oak.poll()
                self.timestamp = get_timestamp_utc().timestamp()

                if q_display is not None and not self.eval_mode:
                    try:
                        p: FramePacket = q_display.get_queue().get(block=False)

                        left_frame = cv2.cvtColor(
                            p[self.sources[self.display_config.key]["left"]].frame, cv2.COLOR_GRAY2RGB
                        )
                        right_frame = cv2.cvtColor(
                            p[self.sources[self.display_config.key]["right"]].frame, cv2.COLOR_GRAY2RGB
                        )
                        self.display.put({"left": left_frame, "right": right_frame}, marker=self.is_recording.is_set())
                    except queue.Empty:
                        pass
                    except Exception as e:
                        logger.exception(e)

                for key, val in self.cameras.items():
                    _, _, q_obs = val
                    if q_obs is None:
                        continue
                    try:
                        p_obs: FramePacket = q_obs.get_queue().get(block=False)
                        if self.is_recording.is_set() or self.eval_mode:
                            # logger.info(f"FPS: {q_obs.get_fps()}")
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
                                rgb_frame = cv2.cvtColor(p_obs[self.sources[key]["rgb"]].frame, cv2.COLOR_BGR2RGB)
                                depth_frame = p_obs[self.sources[key]["depth"]].frame
                            else:
                                rgb_frame = cv2.cvtColor(p_obs.frame, cv2.COLOR_BGR2RGB)
                                depth_frame = None

                            if not self.eval_mode and self.recorder is not None:
                                self.recorder.put(
                                    {"rgb": rgb_frame, "depth": depth_frame},
                                    self.frame_id,
                                    os.path.join(self.video_path, key),
                                    timestamp=self.timestamp,
                                )
                            elif self.eval_mode:
                                self.current_frames[key]["rgb"] = rgb_frame
                                self.current_frames[key]["depth"] = depth_frame

                    except queue.Empty:
                        # logger.info("QUEUE EMPTY")
                        pass
                    except Exception as e:
                        logger.exception(e)
                self.frame_id += 1
                taken = time.monotonic() - start
                time.sleep(max(1 / self.fps - taken, 0))

    def grab(self):
        return self.current_frames

    def start(self):
        self.stop_event.clear()

        self.processes = []
        self.processes.append(threading.Thread(target=self.run, daemon=True))
        if not self.eval_mode and self.recorder is not None:
            self.recorder.start()
        for p in self.processes:
            p.start()
        return self

    def _make_camera(self, key: str, serial: str):
        oak = OakCamera(device=serial, args={"xlinkChunkSize": 0})
        stereo_fps = self.fps
        color_fps = self.fps

        if not self.eval_mode and key == self.display_config.key:
            left = oak.create_camera("left", resolution=self.display_config.resolution, fps=stereo_fps)
            right = oak.create_camera("right", resolution=self.display_config.resolution, fps=stereo_fps)
            q_display = oak.queue([left, right], max_size=3).configure_syncing(
                enable_sync=True, threshold_ms=int((1000 / stereo_fps) / 2)
            )
        else:
            left = None
            right = None
            q_display = None

        color = oak.create_camera("CAM_A", resolution=self.camera_config.resolution, fps=color_fps)
        if self.camera_config.resolution == "1080p":
            color.config_color_camera(isp_scale=(2, 3))

        if self.use_depth:
            stereo = oak.create_stereo(
                left=left, right=right, resolution=self.display_config.resolution, fps=stereo_fps
            )
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

        self.sources[key] = {
            "rgb": color,
            "depth": stereo,
            "left": left,
            "right": right,
        }
        return oak, q_display, q_obs

    def close(self):
        self.stop_event.set()
        if self.recorder:
            self.recorder.stop()

        for oak, _, _ in self.cameras.values():
            if oak is not None:
                oak.close()
        if self.processes:
            for p in self.processes.reverse():
                p.join()


if __name__ == "__main__":
    cams = CameraMultiOak(
        keys={"top": "xxx"},
        # keys={"top": "14442C10114BBCD600", "waist": "14442C10B1E3BCD600"},
        camera_config=CameraOakConfig(fps=30, width=1280, height=800, use_depth=True, resolution="800p"),
        display_config=DisplayConfig(key="top", mode="stereo", crop_sizes=(0, 0, 0, 0), resolution="400p"),
        save_processes=6,
        save_threads=6,
        save_queue_size=120,
    )

    try:
        cams.start()
        time.sleep(0.2)

        cams.start_recording("/tmp/test")
        time.sleep(15)
        cams.stop_recording()
        time.sleep(1)
        cams.close()
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt")
        cams.close()
    except Exception as e:
        logger.exception(e)
        cams.close()
