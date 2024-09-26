from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Event, Queue, shared_memory
from pathlib import Path
from threading import Lock

import cv2
import h5py
import matplotlib.pyplot as plt
import numpy as np
import typer
from fourier_grx_client import ControlGroup, RobotClient
from loguru import logger
from omegaconf import DictConfig, OmegaConf
from pytransform3d import rotations
from tqdm import tqdm

from silverscreen.drivers.hands import FourierDexHand, InspireDexHand
from silverscreen.filters import OneEuroFilter
from silverscreen.preprocess import VuerPreprocessor
from silverscreen.retarget.hand import HandRetarget
from silverscreen.retarget.robot import Robot
from silverscreen.retarget.robot_wrapper import RobotWrapper
from silverscreen.television import OpenTeleVision
from silverscreen.utils import CERT_DIR, se3_to_xyzquat

from .camera import make_camera

# fmt: off
DEFAULT_INDEX =list(range(12, 32))
DEFAULT_QPOS= np.array([
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    -np.pi / 12, 0.1, 0, -np.pi / 2, 0, 0, 0,
    -np.pi / 12, -0.1, 0, -np.pi / 2, 0, 0, 0

])
# fmt: on


class ReplayRobot(RobotWrapper):
    def __init__(self, config: DictConfig, dt=1 / 60, sim=True, show_fpv=False):
        self.sim = sim
        self.left_hand_prefix = config.hand.prefix_left
        self.right_hand_prefix = config.hand.prefix_right
        self.show_fpv = show_fpv
        config.robot.visualize = True
        super().__init__(config.robot, open_viz=False)

        self.dt = dt

        self.update_display()

        self.hand_retarget = HandRetarget(config.hand.config)

        if not self.sim:
            logger.warning("Real robot mode.")
            self.cam = make_camera("zed", open=True)
            self.client = RobotClient(namespace="gr/daq")
            self.left_hand = FourierDexHand(self.config.hand.ip_left)
            self.right_hand = FourierDexHand(self.config.hand.ip_right)

            with ThreadPoolExecutor(max_workers=2) as executor:
                executor.submit(self.left_hand.init)
                executor.submit(self.right_hand.init)

            logger.info("Init robot client.")
            time.sleep(1.0)
            self.client.set_enable(True)

            time.sleep(1.0)

            self.client.move_joints(DEFAULT_INDEX, DEFAULT_QPOS, degrees=False, duration=1.0)
            self.set_joint_positions([self.config.joint_names[i] for i in DEFAULT_INDEX], DEFAULT_QPOS, degrees=False)

    def observe(self):
        if self.sim:
            logger.warning("Sim mode no observation.")
            return None, None, None
        _, img = self.cam.grab()
        left_image = img[::2, : img.shape[1] // 2]
        right_image = img[::2, img.shape[1] // 2 :]

        cropped_img_shape = (240, 320)
        crop_size_h = 0
        crop_size_w = 1280 - 960
        left_image = left_image[crop_size_h:, crop_size_w:]
        left_image = cv2.resize(left_image, (cropped_img_shape[1], cropped_img_shape[0])).transpose(2, 0, 1)
        right_image = right_image[crop_size_h:, 0:-crop_size_w]
        right_image = cv2.resize(right_image, (cropped_img_shape[1], cropped_img_shape[0])).transpose(2, 0, 1)

        left_hand_qpos = self.left_hand.get_positions()
        right_hand_qpos = self.right_hand.get_positions()

        qpos_robot = np.deg2rad(self.client.joint_positions)
        qpos = np.concatenate(
            (
                qpos_robot[[13, 15, 17]],
                qpos_robot[-14:],
                left_hand_qpos,
                right_hand_qpos,
            )
        )
        return qpos, left_image, right_image

    def step(self, action, left_img, right_img):
        qpos, left_hand_qpos, right_hand_qpos = self._convert_action(action)
        self._set_hand_joints(left_hand_qpos, right_hand_qpos)
        self.q_real = qpos
        self.update_display()

        left_img = left_img.transpose((1, 2, 0))
        right_img = right_img.transpose((1, 2, 0))

        if not self.sim:
            self.client.move_joints("all", qpos, degrees=False)
            self.left_hand.set_positions(action[17:23])
            self.right_hand.set_positions(action[23:29])

        if self.show_fpv:
            img = np.concatenate((left_img, right_img), axis=1)
            plt.cla()
            plt.title("VisionPro View")
            plt.imshow(img, aspect="equal")
            plt.pause(0.001)

    def end(self):
        if self.show_fpv:
            plt.close("all")
        if not self.sim:
            self.client.move_joints(
                DEFAULT_INDEX,
                DEFAULT_QPOS,
                degrees=False,
                duration=1.0,
            )
            with ThreadPoolExecutor(max_workers=2) as executor:
                executor.submit(self.left_hand.reset)
                executor.submit(self.left_hand.reset)

    def _convert_action(self, action):
        assert len(action) == 29
        qpos = np.zeros(32)

        qpos[[13, 15, 17]] = action[[0, 1, 2]]
        qpos[-14:] = action[3:17]
        left_hand_qpos, right_hand_qpos = self.hand_retarget.real_to_qpos(action[17:23], action[23:29])

        return qpos, left_hand_qpos, right_hand_qpos

    def _set_hand_joints(self, left_hand_qpos, right_hand_qpos):
        left = np.zeros(11)
        right = np.zeros(11)

        left[0:2] = left_hand_qpos[0]
        left[2:4] = left_hand_qpos[1]
        left[4:6] = left_hand_qpos[3]
        left[6:8] = left_hand_qpos[2]
        left[8] = left_hand_qpos[5]
        left[9:11] = left_hand_qpos[4]

        right[0:2] = right_hand_qpos[0]
        right[2:4] = right_hand_qpos[1]
        right[4:6] = right_hand_qpos[3]
        right[6:8] = right_hand_qpos[2]
        right[8] = right_hand_qpos[5]
        right[9:11] = right_hand_qpos[4]

        # print(left, right, self.left_retargeting.joint_names)
        self.set_joint_positions(
            [
                self.left_hand_prefix + name if not name.startswith(self.left_hand_prefix) else name
                for name in self.hand_retarget.left_joint_names
            ],
            left,
        )
        self.set_joint_positions(
            [
                self.right_hand_prefix + name if not name.startswith(self.right_hand_prefix) else name
                for name in self.hand_retarget.right_joint_names
            ],
            right,
        )


class TeleopRobot(Robot):
    image_queue = Queue()
    toggle_streaming = Event()
    image_lock = Lock()

    def __init__(self, config: DictConfig, dt=1 / 60, sim=True, show_fpv=False):
        super().__init__(config)

        self.sim = sim
        self.left_hand_prefix = config.hand.prefix_left
        self.right_hand_prefix = config.hand.prefix_right

        self.hand_filter = OneEuroFilter(min_cutoff=config.hand_filter.min_cutoff, beta=config.hand_filter.beta)
        self.joint_filter = OneEuroFilter(min_cutoff=config.joint_filter.min_cutoff, beta=config.joint_filter.beta)

        self.hand_retarget = HandRetarget(config.hand.config)

        if not self.sim:
            logger.warning("Real robot mode.")
            self.cam = make_camera("zed", open=True)
            self.client = RobotClient(namespace="gr/daq")
            
            
            logger.info("Init robot client.")
            time.sleep(1.0)
            self.client.set_enable(True)

            time.sleep(1.0)

            self.client.move_joints(DEFAULT_INDEX, positions=DEFAULT_QPOS, degrees=False, duration=1.0)
            self.set_joint_positions([self.config.joint_names[i] for i in DEFAULT_INDEX], DEFAULT_QPOS, degrees=False)
            self.set_posture_target_from_current_configuration()
            
            
            logger.info("Init hands.")
            if self.hand_retarget.hand_type == "fourier":
                self.left_hand = FourierDexHand(config.hand.ip_left)
                self.right_hand = FourierDexHand(config.hand.ip_right)

                with ThreadPoolExecutor(max_workers=2) as executor:
                    executor.submit(self.left_hand.init)
                    executor.submit(self.right_hand.init)
            elif self.hand_retarget.hand_type == "inspire":
                self.left_hand = InspireDexHand(self.config.hand.ip_left)
                self.right_hand = InspireDexHand(self.config.hand.ip_right)
                with ThreadPoolExecutor(max_workers=2) as executor:
                    executor.submit(self.left_hand.reset)
                    executor.submit(self.right_hand.reset)

            

        self.shm = shared_memory.SharedMemory(
            create=True,
            size=np.prod(self.cam.img_shape) * np.uint8().itemsize,  # type: ignore
        )

        print(self.cam.img_shape, self.cam.resolution)
        self.img_array = np.ndarray(
            self.cam.img_shape,
            dtype=np.uint8,
            buffer=self.shm.buf,
        )

        self.tv = OpenTeleVision(
            self.cam.resolution,
            self.shm.name,
            self.image_queue,
            stream_mode="image",
            toggle_streaming=self.toggle_streaming,
            ngrok=False,
            cert_file=str(CERT_DIR / "cert.pem"),
            key_file=str(CERT_DIR / "key.pem"),
        )

        self.processor = VuerPreprocessor(hand_type=self.hand_retarget.hand_type)

    def update_image(self, gray=False):
        timestamp, rgb = self.cam.grab(gray)
        # https://gist.github.com/bhawkins/5095558
        with self.image_lock:
            np.copyto(self.img_array, rgb)
        return timestamp

    def start_recording(self, output_path: str):
        self.cam.start_recording(output_path)

    def stop_recording(self):
        if self.cam.is_recording:
            self.cam.stop_recording()

    def step(self):
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)

        left_pose = np.concatenate(
            [
                left_wrist_mat[:3, 3],
                rotations.quaternion_from_matrix(left_wrist_mat[:3, :3])[[1, 2, 3, 0]],
            ]
        )
        right_pose = np.concatenate(
            [
                right_wrist_mat[:3, 3],
                rotations.quaternion_from_matrix(right_wrist_mat[:3, :3])[[1, 2, 3, 0]],
            ]
        )

        left_qpos, right_qpos = self.hand_retarget.retarget(left_hand_mat, right_hand_mat)

        return (
            head_mat,
            left_pose,
            right_pose,
            left_qpos,
            right_qpos,
        )

    def end(self):
        if not self.sim:
            self.client.move_joints(DEFAULT_INDEX, positions=DEFAULT_QPOS, degrees=False, duration=1.0)
            with ThreadPoolExecutor(max_workers=2) as executor:
                executor.submit(self.left_hand.reset)
                executor.submit(self.left_hand.reset)

    def set_display_hand_joints(self, left_hand_qpos, right_hand_qpos):
        left = np.zeros(11)
        right = np.zeros(11)

        left[0:2] = left_hand_qpos[0]
        left[2:4] = left_hand_qpos[1]
        left[4:6] = left_hand_qpos[3]
        left[6:8] = left_hand_qpos[2]
        left[8] = left_hand_qpos[5]
        left[9:11] = left_hand_qpos[4]

        right[0:2] = right_hand_qpos[0]
        right[2:4] = right_hand_qpos[1]
        right[4:6] = right_hand_qpos[3]
        right[6:8] = right_hand_qpos[2]
        right[8] = right_hand_qpos[5]
        right[9:11] = right_hand_qpos[4]

        # print(left, right, self.left_retargeting.joint_names)
        self.set_joint_positions(
            [
                self.left_hand_prefix + name if not name.startswith(self.left_hand_prefix) else name
                for name in self.hand_retarget.left_joint_names
            ],
            left,
        )
        self.set_joint_positions(
            [
                self.right_hand_prefix + name if not name.startswith(self.right_hand_prefix) else name
                for name in self.hand_retarget.right_joint_names
            ],
            right,
        )

    def get_ee_pose(self):
        if self.sim:
            return None, None
        left_ee_pose = self.client.get_transform("l_hand_pitch", "base")
        right_ee_pose = self.client.get_transform("r_hand_pitch", "base")
        left_ee_pose = se3_to_xyzquat(left_ee_pose)
        right_ee_pose = se3_to_xyzquat(right_ee_pose)

        return np.hstack([left_ee_pose, right_ee_pose])

    def control_hands(self, left_qpos, right_qpos):
        left_qpos_real, right_qpos_real = self.hand_retarget.qpos_to_real(left_qpos, right_qpos)
        if self.sim:
            return np.hstack([left_qpos_real, right_qpos_real])
        if self.hand_retarget.hand_type == "inspire":
            filtered_hand_qpos = self.hand_filter.next(
                time.time(),
                np.hstack([left_qpos_real, right_qpos_real]),
            ).astype(int)
        elif self.hand_retarget.hand_type == "fourier":
            filtered_hand_qpos = self.hand_filter.next(
                time.time(),
                np.hstack([left_qpos_real, right_qpos_real]),
            )
        else:
            raise ValueError("Invalid hand type.")
        self.left_hand.set_positions(filtered_hand_qpos[:6], wait_reply=False)
        self.right_hand.set_positions(filtered_hand_qpos[6:], wait_reply=False)

        return filtered_hand_qpos

    def control_joints(self):
        qpos = self.q_real
        # qpos = np.rad2deg(qpos)
        qpos = self.joint_filter.next(time.time(), qpos)
        self.client.move_joints(ControlGroup.ALL, qpos, degrees=False)

        return qpos

    def observe(self, degrees=False):
        if self.sim:
            return np.zeros(32), np.zeros(32), np.zeros(6)

        qpos = self.client.joint_positions
        qvel = self.client.joint_velocities
        left_qpos, right_qpos = self.left_hand.get_positions(), self.right_hand.get_positions()
        hand_qpos = np.hstack([left_qpos, right_qpos])
        # TODO: this might change
        if degrees:
            return qpos, qvel, hand_qpos
        return np.deg2rad(qpos), np.deg2rad(qvel), hand_qpos


def main(data_dir: str, task: str = "01_cube_kitting", episode: int = 1, config: str = "config.yml"):
    root = data_dir
    folder_name = f"{task}/processed"
    episode_name = f"processed_episode_{episode}.hdf5"
    episode_path = Path(root) / folder_name / episode_name

    data = h5py.File(str(episode_path), "r")
    actions = np.array(data["qpos_action"])[::2]
    left_imgs = np.array(data["observation.image.left"])[::2]  # 30hz
    right_imgs = np.array(data["observation.image.right"])[::2]
    data.close()

    timestamps = actions.shape[0]

    replay_robot = ReplayRobot(OmegaConf.load(config), dt=1 / 30, show_fpv=True)

    try:
        for t in tqdm(range(timestamps)):
            replay_robot.step(actions[t], left_imgs[t, :], right_imgs[t, :])
    except KeyboardInterrupt:
        replay_robot.end()
        exit()


if __name__ == "__main__":
    typer.run(main)
