import multiprocessing as mp
from multiprocessing import shared_memory
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from loguru import logger
from depthai_sdk import OakCamera, RecordType
from depthai_sdk.classes.packets import FramePacket
from depthai_sdk.classes.packet_handlers import RecordPacketHandler
from depthai_sdk.record import RecordType, Record
import depthai as dai

from silverscreen.camera.utils import save_images_threaded
from silverscreen.utils import get_timestamp_utc
from .camera_base import CameraBase


class CamOak(CameraBase):
    def __init__(
        self,
        index: int = 0,
        fps: int = 60,
    ):
        super().__init__(index, fps, (720, 1280))
        self.oak = None
        self.save_queue = mp.Queue(maxsize=30)
        self.episode_id = 0
        self.frame_id = 0

    def start(self):
        self.stop_event.clear()
        self.processes = []
        self.processes.append(threading.Thread(target=self.run, daemon=True))
        self.processes.append(mp.Process(target=save_images_threaded, args=(self.save_queue, 4)))
        for p in self.processes:
            p.start()
        # self.run()
        return self

    def _make_camera(self):
        if self.oak is not None:
            self.oak.close()
        oak = OakCamera(args={"irDotBrightness": 500})
        left = oak.create_camera("left", resolution="720p", fps=60)
        right = oak.create_camera("right", resolution="720p", fps=60)
        q_display = (
            oak.queue([left, right], max_size=3).configure_syncing(threshold_ms=int((1000 / 60) / 2)).get_queue()
        )

        color = oak.create_camera("CAM_A", resolution="1080p", encode="mjpeg", fps=30)
        color.config_color_camera(isp_scale=(2, 3))
        stereo = oak.create_stereo(left=left, right=right, resolution="720p", fps=30)
        stereo.config_stereo(align=color, subpixel=True, lr_check=True)
        # stereo.node.setOutputSize(640, 360) # 720p, downscaled to 640x360 (decimation filter, median filtering)
        # On-device post processing for stereo depth
        config = stereo.node.initialConfig.get()
        stereo.node.setPostProcessingHardwareResources(3, 3)
        config.postProcessing.speckleFilter.enable = True
        config.postProcessing.thresholdFilter.minRange = 400
        config.postProcessing.thresholdFilter.maxRange = 3_000  # 3m
        config.postProcessing.decimationFilter.decimationFactor = 2
        config.postProcessing.decimationFilter.decimationMode = (
            dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.NON_ZERO_MEDIAN
        )
        stereo.node.initialConfig.set(config)

        q_obs = oak.queue([color, stereo], max_size=30).configure_syncing(threshold_ms=int((1000 / 30) / 2)).get_queue()

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

    def run(self):
        logger.info("Starting Oak camera")

        self._make_camera()

        # def make_new_record():
        #     record_components = [self.sources["depth"].out.depth, self.sources["rgb"].out.encoded]
        #     record = RecordPacketHandler(record_components, Record(Path(self.video_path).resolve(), RecordType.VIDEO))
        #     record.configure_syncing(True, threshold_ms=500 / 30)
        #     return record

        while not self.stop_event.is_set():
            self.oak.start()
            while self.oak.running():
                start = time.monotonic()
                self.oak.poll()
                self.timestamp = get_timestamp_utc().timestamp()

                try:
                    p: FramePacket = self.q_display.get(block=False)

                    left_frame = cv2.cvtColor(p[self.sources["left"]].frame, cv2.COLOR_GRAY2RGB)
                    right_frame = cv2.cvtColor(p[self.sources["left"]].frame, cv2.COLOR_GRAY2RGB)
                    self.send_to_display({"left": left_frame, "right": right_frame}, marker=self.flag_marker)
                except:
                    pass

                if self.is_recording:
                    try:
                        p_obs: FramePacket = self.q_obs.get(block=False)
                        rgb_frame = cv2.cvtColor(p_obs[self.sources["rgb"]].frame, cv2.COLOR_BGR2RGB)
                        depth_frame = p_obs[self.sources["depth"]].frame
                        self.send_to_obs({"rgb": rgb_frame, "depth": depth_frame})
                    except:
                        pass

                taken = time.monotonic() - start
                time.sleep(max(1 / self.fps - taken, 0))

                # logger.warning(f"Left frame shape: {left_frame.shape}")
                # logger.warning(f"Right frame shape: {right_frame.shape}")

    def send_to_obs(self, frames: dict[str, np.ndarray]):
        for key, frame in frames.items():
            self.save_queue.put((frame, key, self.frame_id, self.video_path))
        self.frame_id += 1

    def start_recording(self, output_path: str):
        self.frame_id = 0
        return super().start_recording(output_path)

    def stop_recording(self):
        self.frame_id = 0
        return super().stop_recording()

    def grab(self, sources: list[str]) -> tuple[float, dict[str, np.ndarray]]:
        self.oak.poll()
        p: FramePacket = self.q_obs.get(block=True)
        return p[self.sources["rgb"]], p[self.sources["depth"]]

    def close(self):
        self.stop_event.set()
        if self.save_queue is not None:
            self.save_queue.put(None)
        if self.oak is not None:
            self.oak.close()
        if self.processes is not None:
            for p in self.processes.reverse():
                p.join()
