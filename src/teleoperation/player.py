from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Event, Queue
from threading import Lock
from typing import Any, Literal

import hydra
import numpy as np
from omegaconf import DictConfig

from teleoperation.adapter.hands import DummyDexHand, HandAdapter
from teleoperation.adapter.robots import DummyRobot, RobotAdapter
from teleoperation.camera.utils import post_process
from teleoperation.preprocess import VuerPreprocessor
from teleoperation.retarget.robot import DexRobot
from teleoperation.television import OpenTeleVision
from teleoperation.upsampler import Upsampler
from teleoperation.utils import CERT_DIR, se3_to_xyzortho6d

logger = logging.getLogger(__name__)


class CameraMixin:
    cam: Any  # TODO: defnine a protocol for this
    sim: bool

    def observe_vision(self, mode: Literal["stereo", "rgbd"] = "stereo", resolution: tuple[int, int] = (240, 320)):
        if self.sim:
            logger.warning("Sim mode no observation.")
            return None

        if mode == "stereo":
            sources = ["left", "right"]
        elif mode == "rgbd":
            sources = ["left", "depth"]
        else:
            raise ValueError("Invalid mode.")

        _, image_dict = self.cam.grab(sources=sources)
        image_dict = post_process(image_dict, resolution, (0, 0, 0, 1280 - 960))

        images = None
        if mode == "stereo":
            left_image = image_dict["left"].transpose(2, 0, 1)
            right_image = image_dict["right"].transpose(2, 0, 1)
            images = (left_image, right_image)

        elif mode == "rgbd":
            left_image = image_dict["left"].transpose(2, 0, 1)
            depth_image = image_dict["depth"].transpose(2, 0, 1)
            images = (left_image, depth_image)
        else:
            raise ValueError("Invalid mode.")

        return images


# class ReplayRobot(DexRobot, CameraMixin):
#     def __init__(self, cfg: DictConfig, dt=1 / 60, sim=True, show_fpv=False):
#         self.sim = sim
#         self.show_fpv = show_fpv
#         cfg.robot.visualize = True
#         super().__init__(cfg)

#         self.dt = dt

#         self.update_display()

#         if not self.sim:
#             logger.warning("Real robot mode.")
#             self.cam = (
#                 hydra.utils.instantiate(cfg.camera.instance)
#                 .with_display(cfg.camera.display.mode, cfg.camera.display.resolution, cfg.camera.display.crop_sizes)
#                 .start()
#             )
#             self.client = RobotClient(namespace="gr/daq")
#             self.left_hand = FourierDexHand(self.config.hand.ip_left)
#             self.right_hand = FourierDexHand(self.config.hand.ip_right)

#             with ThreadPoolExecutor(max_workers=2) as executor:
#                 executor.submit(self.left_hand.init)
#                 executor.submit(self.right_hand.init)

#             logger.info("Init robot client.")
#             time.sleep(1.0)
#             self.client.set_enable(True)

#             time.sleep(1.0)

#             self.client.move_joints(
#                 self.config.controlled_joint_indices, self.config.default_qpos, degrees=False, duration=1.0
#             )
#             self.set_joint_positions(
#                 [self.config.joint_names[i] for i in self.config.controlled_joint_indices],
#                 self.config.default_qpos,
#                 degrees=False,
#             )

#     def observe(self):
#         if self.sim:
#             logger.warning("Sim mode no observation.")
#             return None, None, None

#         qpos = self.client.joint_positions
#         # qvel = self.client.joint_velocities
#         left_qpos, right_qpos = self.left_hand.get_positions(), self.right_hand.get_positions()
#         left_qpos, right_qpos = self.hand_retarget.real_to_qpos(left_qpos, right_qpos)
#         hand_qpos = np.hstack([left_qpos, right_qpos])
#         ee_pose = get_ee_pose(self.client)
#         head_pose = get_head_pose(self.client)

#         return qpos, hand_qpos, ee_pose, head_pose

#     def step(self, action, left_img, right_img):
#         qpos, left_hand_real, right_hand_real = self._convert_action(action)
#         self.set_hand_joints(left_hand_real, right_hand_real)
#         self.q_real = qpos
#         self.update_display()

#         if not self.sim:
#             self.client.move_joints("all", qpos, degrees=False)
#             self.left_hand.set_positions(action[17:23])
#             self.right_hand.set_positions(action[23:29])

#         if self.show_fpv:
#             left_img = left_img.transpose((1, 2, 0))
#             right_img = right_img.transpose((1, 2, 0))
#             img = np.concatenate((left_img, right_img), axis=1)
#             plt.cla()
#             plt.title("VisionPro View")
#             plt.imshow(img, aspect="equal")
#             plt.pause(0.001)

#     def end(self):
#         if self.show_fpv:
#             plt.close("all")
#         if not self.sim:
#             self.client.move_joints(
#                 self.config.controlled_joint_indices,
#                 self.config.default_qpos,
#                 degrees=False,
#                 duration=1.0,
#             )
#             with ThreadPoolExecutor(max_workers=2) as executor:
#                 executor.submit(self.left_hand.reset)
#                 executor.submit(self.left_hand.reset)

#     def _convert_action(self, action):
#         assert len(action) == 29
#         qpos = np.zeros(32)

#         qpos[[13, 16, 17]] = action[[0, 1, 2]]
#         qpos[-14:] = action[3:17]

#         left_qpos, right_qpos = self.hand_retarget.qpos_to_real(action[17:23], action[23:29])

#         return qpos, left_qpos, right_qpos


class TeleopRobot(DexRobot, CameraMixin):
    image_queue = Queue()
    toggle_streaming = Event()
    image_lock = Lock()

    def __init__(self, cfg: DictConfig, how_fpv=False):
        super().__init__(cfg)

        self.sim = cfg.sim
        self.dt = 1 / cfg.frequency

        # update joint positions in pinocchio
        self.set_joint_positions(
            [self.config.joint_names[i] for i in self.config.controlled_joint_indices],
            self.config.default_qpos,
            degrees=False,
        )
        self.set_posture_target_from_current_configuration()

        self.cam = hydra.utils.instantiate(cfg.camera.instance).start()

        self.tv = OpenTeleVision(
            self.cam.display.shape,
            self.cam.display.shm_name,
            stream_mode=f"rgb_{self.cam.display.mode}",
            ngrok=False,
            cert_file=str(CERT_DIR / "cert.pem"),
            key_file=str(CERT_DIR / "key.pem"),
        )

        self.processor = VuerPreprocessor(cfg.preprocessor)

        if not self.sim:
            logger.warning("Real robot mode.")
            self.client: RobotAdapter = hydra.utils.instantiate(cfg.robot.instance)

            self.client.connect()
            self.upsampler = Upsampler(
                self.client,
                target_hz=cfg.upsampler.frequency,
                initial_command=self.client.joint_positions,
                gravity_compensation=cfg.upsampler.gravity_compensation,
            )
            self.upsampler.start()

            logger.info("Init hands.")
            self.left_hand: HandAdapter = hydra.utils.instantiate(cfg.hand.left_hand)
            self.right_hand: HandAdapter = hydra.utils.instantiate(cfg.hand.right_hand)

            if self.hand_retarget.hand_type == "inspire":
                with ThreadPoolExecutor(max_workers=2) as executor:
                    executor.submit(self.left_hand.reset)
                    executor.submit(self.right_hand.reset)
        else:
            self.client: RobotAdapter = DummyRobot(32)
            self.upsampler = Upsampler(self.client, target_hz=cfg.upsampler.frequency)  # TODO: dummy robot
            self.upsampler.start()
            self.left_hand: HandAdapter = DummyDexHand(6)
            self.right_hand: HandAdapter = DummyDexHand(6)

    def start_recording(self, output_path: str):
        self.cam.start_recording(output_path)

    def stop_recording(self):
        if self.cam.is_recording:
            self.cam.stop_recording()

    def step(self):
        """Receive measurements from Mocap/VR

        Returns:
            head_mat, left_pose, right_pose, left_hand_qpos, right_hand_qpos,
        """
        head_mat, left_wrist_mat, right_wrist_mat, left_hand_mat, right_hand_mat = self.processor.process(self.tv)

        if self.viz and self.config.get("debug_hand", False):
            from itertools import product

            left_wrist_display = left_wrist_mat.copy()
            right_wrist_display = right_wrist_mat.copy()
            left_wrist_display[:3, 3] *= self.config.body_scaling_factor
            right_wrist_display[:3, 3] *= self.config.body_scaling_factor
            self.viz.viewer["left_hand/0"].set_transform(right_wrist_display)
            self.viz.viewer["right_hand/0"].set_transform(right_wrist_display)
            for side, finger in product(["left", "right"], range(25)):
                if side == "left":
                    landmark_tf = np.eye(4)
                    landmark_tf[:3, 3] = left_hand_mat[finger]
                    transform = left_wrist_display @ landmark_tf
                    self.viz.viewer[f"{side}_hand/{finger}"].set_transform(transform)
                else:
                    landmark_tf = np.eye(4)
                    landmark_tf[:3, 3] = right_hand_mat[finger]
                    transform = right_wrist_display @ landmark_tf
                    self.viz.viewer[f"{side}_hand/{finger}"].set_transform(transform)

        # left_pose = se3_to_xyzquat(left_wrist_mat)
        # right_pose = se3_to_xyzquat(right_wrist_mat)

        left_qpos, right_qpos = self.hand_retarget.retarget(left_hand_mat, right_hand_mat)

        return (
            head_mat,
            left_wrist_mat,
            right_wrist_mat,
            left_qpos,
            right_qpos,
        )

    def observe(self):
        left_qpos, right_qpos = self.left_hand.get_positions(), self.right_hand.get_positions()
        # left_qpos, right_qpos = self.hand_retarget.real_to_qpos(left_qpos, right_qpos)
        hand_qpos = np.hstack([left_qpos, right_qpos])

        (qpos,) = self.client.observe()

        left_ee_pose, right_ee_pose, head_pose = self._get_ee_pose(qpos)
        ee_pose = np.hstack([left_ee_pose, right_ee_pose])

        return qpos, hand_qpos, ee_pose, head_pose

    def _get_ee_pose(self, qpos):
        left_link = self.config.named_links["left_end_effector_link"]
        right_link = self.config.named_links["right_end_effector_link"]
        head_link = self.config.named_links["head_link"]
        root_link = self.config.named_links["root_link"]

        left_pose = self.frame_placement(qpos, left_link, root_link).homogeneous
        right_pose = self.frame_placement(qpos, right_link, root_link).homogeneous
        head_pose = self.frame_placement(qpos, head_link, root_link).homogeneous

        left_pose = se3_to_xyzortho6d(left_pose)
        right_pose = se3_to_xyzortho6d(right_pose)
        head_pose = se3_to_xyzortho6d(head_pose)  # TODO: should we discard translation?

        return left_pose, right_pose, head_pose

    def control_hands(self, left_qpos: np.ndarray, right_qpos: np.ndarray):
        """Control real hands

        Args:
            left_qpos (np.ndarray): Hand qpos in radians
            right_qpos (np.ndarray): Hand qpos in radians

        Returns:
            np.ndarray: concatenated and filtered hand qpos in real steps
        """
        left, right = self.hand_action_convert(left_qpos, right_qpos, filtering=True)
        self.left_hand.set_positions(left)
        self.right_hand.set_positions(right)

        return np.hstack([left, right])

    def control_joints(self):
        qpos = self.joint_filter.next(time.time(), self.q_real)
        self.upsampler.put(qpos)
        return qpos

    def init_control_joints(self):
        self.client.init_command_joints(self.q_real)

    def pause_robot(self):
        logger.info("Pausing robot...")
        self.upsampler.pause()
        # self.client.move_joints(ControlGroup.ALL, self.client.joint_positions, gravity_compensation=False)

    def end(self):
        self.upsampler.stop()
        self.upsampler.join()
        self.client.disconnect()
        with ThreadPoolExecutor(max_workers=2) as executor:
            executor.submit(self.left_hand.reset)
            executor.submit(self.left_hand.reset)


# def main(data_dir: str, task: str = "01_cube_kitting", episode: int = 1, config: str = "config.yml"):
#     root = data_dir
#     folder_name = f"{task}/processed"
#     episode_name = f"processed_episode_{episode}.hdf5"
#     episode_path = Path(root) / folder_name / episode_name

#     data = h5py.File(str(episode_path), "r")
#     actions = np.array(data["qpos_action"])[::2]
#     left_imgs = np.array(data["observation.image.left"])[::2]  # 30hz
#     right_imgs = np.array(data["observation.image.right"])[::2]
#     data.close()

#     timestamps = actions.shape[0]

#     replay_robot = ReplayRobot(OmegaConf.load(config), dt=1 / 30, show_fpv=True)

#     try:
#         for t in tqdm(range(timestamps)):
#             replay_robot.step(actions[t], left_imgs[t, :], right_imgs[t, :])
#     except KeyboardInterrupt:
#         replay_robot.end()
#         exit()


# if __name__ == "__main__":
#     typer.run(main)
