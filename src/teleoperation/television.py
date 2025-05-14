# Copyright [2024] [Xuxin Cheng, Jialong Li, Shiqi Yang, Ge Yang and Xiaolong Wang]

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ------------------

# This code builds upon following open-source code-bases. Please visit the URLs to see the respective LICENSES:

# 1) https://github.com/tonyzhaozh/act
# 2) https://github.com/facebookresearch/detr
# 3) https://github.com/dexsuite/dex-retargeting
# 4) https://github.com/vuer-ai/vuer

# ------------------

import asyncio
import logging
import multiprocessing as mp
import time
from multiprocessing.shared_memory import SharedMemory
from threading import Lock
from typing import Literal

import numpy as np
from vuer import Vuer
from vuer.schemas import Hands, ImageBackground

logger = logging.getLogger(__name__)
image_lock = Lock()


def get_hostname():
    import os

    return os.uname().nodename


class OpenTeleVision:
    def __init__(
        self,
        img_shape,
        shm_name,
        stream_mode: Literal["rgb_mono", "rgb_stereo"] = "rgb_stereo",
        cert_file="./cert.pem",
        key_file="./key.pem",
        ngrok=False,
    ):
        self._connected = mp.Value("b", False, lock=True)

        self.stream_mode = stream_mode
        if self.stream_mode == "rgb_stereo":
            self.img_shape = (img_shape[0], img_shape[1] * 2, 3)
        else:
            self.img_shape = (img_shape[0], img_shape[1], 3)
        self.img_height, self.img_width = img_shape[:2]

        if ngrok:
            url = "http://localhost:8012"
            ws_url = "ws://localhost:8012"
            self.app = Vuer(
                domain=url,
                host="0.0.0.0",
                queries={"ws": ws_url, "grid": False},
                queue_len=3,
            )
        else:
            url = f"https://{get_hostname()}.local:8012"
            ws_url = f"wss://{get_hostname()}.local:8012"
            self.app = Vuer(
                domain=url,
                host="0.0.0.0",
                cert=cert_file,
                key=key_file,
                queries={"ws": ws_url, "grid": False},
                queue_len=3,
            )

        self.app.add_handler("HAND_MOVE")(self.on_hand_move)  # type: ignore
        self.app.add_handler("CAMERA_MOVE")(self.on_cam_move)  # type: ignore

        if stream_mode.startswith("rgb"):
            existing_shm = SharedMemory(name=shm_name)
            self.img_array = np.ndarray(
                (self.img_shape[0], self.img_shape[1], 3),
                dtype=np.uint8,
                buffer=existing_shm.buf,
            )
            self.app.spawn(start=False)(self.main_image)  # type: ignore
        else:
            raise ValueError("stream_mode must be either 'rgb_mono' or 'rgb_stereo'")

        self.left_wrist_shared = mp.Array("d", 16)
        self.right_wrist_shared = mp.Array("d", 16)
        self.left_landmarks_shared = mp.Array("d", 25 * 3)
        self.right_landmarks_shared = mp.Array("d", 25 * 3)

        self.head_matrix_shared = mp.Array("d", 16)
        self.aspect_shared = mp.Value("d", 1.0)
        self.process = mp.Process(target=self.run)
        self.process.daemon = True
        self.process.start()

    @property
    def connected(self):
        with self._connected.get_lock():
            return bool(self._connected.value)

    def run(self):
        self.app.run()
        logger.error("App exited unexpectedly")

    async def on_cam_move(self, event, session, fps=60):
        try:
            self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            self.aspect_shared.value = event.value["camera"]["aspect"]
        except Exception as e:
            logger.debug(f"Error in on_cam_move: {e}")
            pass

    async def on_hand_move(self, event, session, fps=60):
        try:
            self.left_wrist_shared[:] = np.array(event.value["leftHand"]).flatten()
            self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
            if not self._connected.value:
                self._connected.value = True
                logger.info("first hand received")

        except Exception as e:
            logger.debug(f"Error in on_hand_move left: {e}")
            pass

        try:
            self.right_wrist_shared[:] = np.array(event.value["rightHand"]).flatten()
            self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
            if not self._connected.value:
                self._connected.value = True
                logger.info("first hand received")

        except Exception as e:
            logger.debug(f"Error in on_hand_move right: {e}")
            logger.debug(f"{len(event.value['rightHand'])=} {len(event.value['rightLandmarks'])=}")
            pass

    async def main_image(self, session, fps=30):
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=False, showRight=False)  # type: ignore
        end_time = time.time()
        while True:
            start = time.time()
            image_lock.acquire()
            display_image = self.img_array

            if self.stream_mode == "rgb_mono":
                session.upsert(
                    ImageBackground(
                        # Can scale the images down.
                        display_image[::2, :],
                        # 'jpg' encoding is significantly faster than 'png'.
                        format="jpeg",
                        quality=80,
                        key="left-image",
                        interpolate=True,
                        # fixed=True,
                        aspect=1.778,
                        distanceToCamera=2,
                        position=[0, -0.5, -2],
                        rotation=[0, 0, 0],
                    ),
                    to="bgChildren",
                )
            elif self.stream_mode == "rgb_stereo":
                session.upsert(
                    [
                        ImageBackground(
                            # Can scale the images down.
                            display_image[::2, : self.img_width],
                            # display_image[:self.img_height:2, ::2],
                            # 'jpg' encoding is significantly faster than 'png'.
                            format="jpeg",
                            quality=80,
                            key="left-image",
                            interpolate=True,
                            # fixed=True,
                            aspect=1.66667,
                            # distanceToCamera=0.5,
                            height=8,
                            position=[0, -1, 3],
                            # rotation=[0, 0, 0],
                            layers=1,
                            alphaSrc="./vinette.jpg",
                        ),
                        ImageBackground(
                            # Can scale the images down.
                            display_image[::2, self.img_width :],
                            # display_image[self.img_height::2, ::2],
                            # 'jpg' encoding is significantly faster than 'png'.
                            format="jpeg",
                            quality=80,
                            key="right-image",
                            interpolate=True,
                            # fixed=True,
                            aspect=1.66667,
                            # distanceToCamera=0.5,
                            height=8,
                            position=[0, -1, 3],
                            # rotation=[0, 0, 0],
                            layers=2,
                            alphaSrc="./vinette.jpg",
                        ),
                    ],
                    to="bgChildren",
                )
            # rest_time = 1/fps - time.time() + start
            end_time = time.time()
            image_lock.release()

            # print(f"fps: {1 / (end_time - start)}")

            sleep_time = max(1 / fps - (end_time - start), 0)
            await asyncio.sleep(sleep_time)

    @property
    def left_wrist(self):
        return np.array(self.left_wrist_shared[:], dtype=float).reshape(4, 4, order="F")

    @property
    def right_wrist(self):
        return np.array(self.right_wrist_shared[:], dtype=float).reshape(4, 4, order="F")

    @property
    def left_landmarks(self):
        return np.array(self.left_landmarks_shared[:], dtype=float).reshape(25, 3)

    @property
    def right_landmarks(self):
        return np.array(self.right_landmarks_shared[:], dtype=float).reshape(25, 3)

    @property
    def head_matrix(self):
        return np.array(self.head_matrix_shared[:], dtype=float).reshape(4, 4, order="F")

    @property
    def aspect(self):
        return float(self.aspect_shared.value)
