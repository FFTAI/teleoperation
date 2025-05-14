from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Event, Queue
from threading import Lock
from typing import Any, Literal

import cv2
import hydra
import numpy as np
from omegaconf import DictConfig

from teleoperation.adapter.hands import DummyDexHand, HandAdapter
from teleoperation.adapter.robots import DummyRobot, RobotAdapter
from teleoperation.camera.utils import create_colored_point_cloud_from_depth_oak, post_process
from teleoperation.preprocess import VuerPreprocessor
from teleoperation.retarget.robot import DexRobot
from teleoperation.television import OpenTeleVision
from teleoperation.upsampler import Upsampler
from teleoperation.utils import CERT_DIR, se3_to_xyzortho6d

logger = logging.getLogger(__name__)


try:
    import torch
except ImportError:
    logger.warning("Torch not installed.")
    torch = None

try:
    import rerun as rr

except ImportError:
    logger.warning("Rerun not installed.")
    rr = None


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

        # disable https if mocap method is not avp
        use_http = cfg.get("mocap", "avp") != "avp"

        self.tv = OpenTeleVision(
            self.cam.display.shape,
            self.cam.display.shm_name,
            stream_mode=f"rgb_{self.cam.display.mode}",  # type: ignore
            ngrok=use_http,
            cert_file=str(CERT_DIR / "cert.pem"),
            key_file=str(CERT_DIR / "key.pem"),
        )

        self.processor = VuerPreprocessor(cfg.preprocessor)

        self._init_command_sent = False

        if not self.sim:
            logger.warning("Real robot mode.")

            self.client: RobotAdapter = hydra.utils.instantiate(cfg.robot.instance)

            self.client.connect()
            self.upsampler = Upsampler(
                self.client,
                target_hz=cfg.upsampler.frequency,
                dimension=cfg.robot.num_joints,
                initial_command=self.client.joint_positions,
                gravity_compensation=cfg.upsampler.gravity_compensation,
            )

            logger.info("Init hands.")
            self.left_hand: HandAdapter = hydra.utils.instantiate(cfg.hand.left_hand)
            self.right_hand: HandAdapter = hydra.utils.instantiate(cfg.hand.right_hand)

            if self.hand_retarget.hand_type == "inspire":
                with ThreadPoolExecutor(max_workers=2) as executor:
                    executor.submit(self.left_hand.reset)
                    executor.submit(self.right_hand.reset)
        else:
            self.client: RobotAdapter = DummyRobot(
                cfg.robot.num_joints,
            )
            self.upsampler = Upsampler(
                self.client, dimension=cfg.robot.num_joints, target_hz=cfg.upsampler.frequency
            )  # TODO: dummy robot
            # self.upsampler.start()
            hand_dimension = cfg.hand.left_hand.get("dimension", 6)
            self.left_hand: HandAdapter = DummyDexHand(hand_dimension)
            self.right_hand: HandAdapter = DummyDexHand(hand_dimension)

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

        if self.viz and self.config.get("debug_hand", False):  # type: ignore
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
        left_qpos, right_qpos = self.hand_retarget.real_to_qpos(left_qpos, right_qpos)
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

    def control_hands(self, left_qpos: np.ndarray, right_qpos: np.ndarray, return_real=False):
        """Control real hands

        Args:
            left_qpos (np.ndarray): Hand qpos in radians
            right_qpos (np.ndarray): Hand qpos in radians

        Returns:
            np.ndarray: concatenated and filtered hand qpos in real steps
        """
        left, right = self.hand_action_convert(left_qpos, right_qpos, filtering=True)
        self.left_hand.set_positions(left)  # type: ignore
        self.right_hand.set_positions(right)  # type: ignore

        if return_real:
            return np.hstack([left, right])
        else:
            actuated_indices = self.hand_retarget.cfg.actuated_indices
            return np.hstack([left_qpos[actuated_indices], right_qpos[actuated_indices]])

    def control_joints(self):
        qpos = self.joint_filter.next(time.time(), self.q_real)
        self.upsampler.put(qpos)
        return qpos

    def init_control_joints(self):
        if self._init_command_sent:
            return

        # next_cmd = self.client.init_command_joints(self.q_real)
        self.upsampler.start()
        self.upsampler.put(self.q_real)
        self._init_command_sent = True
        logger.info("Init command sent.")

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

        import os

        os._exit(0)


class RemotePolicy:
    def __init__(self, config: DictConfig):
        self.device = "cpu"
        logger.info(f"Device: {self.device}")

        self.endpoint = config.endpoint

        raise NotImplementedError("Remote policy not implemented yet.")

    def select_action(self, batch):
        raise NotImplementedError("Remote policy not implemented yet.")


class EvalRobot(DexRobot, CameraMixin):
    def __init__(
        self,
        cfg: DictConfig,
    ):
        import rerun as rr

        if cfg.eval.rerun_enabled and rr is not None:
            rr.init("eval_robot", spawn=False)
            rr.connect_tcp(cfg.eval.rerun_endpoint)
            logging.getLogger().addHandler(rr.LoggingHandler("logs/handler"))
        else:
            rr = None

        self.eval_cfg = cfg.eval
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

        self._init_command_sent = False
        self.policy = hydra.utils.instantiate(cfg.policy.instance)
        self._step = 0

        if not self.sim:
            logger.warning("Real robot mode.")

            self.client: RobotAdapter = hydra.utils.instantiate(cfg.robot.instance)

            self.client.connect()
            self.upsampler = Upsampler(
                self.client,
                target_hz=cfg.upsampler.frequency,
                dimension=cfg.robot.num_joints,
                initial_command=self.client.joint_positions,
                gravity_compensation=cfg.upsampler.gravity_compensation,
            )

            logger.info("Init hands.")
            self.left_hand: HandAdapter = hydra.utils.instantiate(cfg.hand.left_hand)
            self.right_hand: HandAdapter = hydra.utils.instantiate(cfg.hand.right_hand)

            if self.hand_retarget.hand_type == "inspire":
                with ThreadPoolExecutor(max_workers=2) as executor:
                    executor.submit(self.left_hand.reset)
                    executor.submit(self.right_hand.reset)
        else:
            self.client: RobotAdapter = DummyRobot(
                cfg.robot.num_joints,
            )
            self.upsampler = Upsampler(
                self.client, dimension=cfg.robot.num_joints, target_hz=cfg.upsampler.frequency
            )  # TODO: dummy robot
            # self.upsampler.start()
            hand_dimension = cfg.hand.left_hand.get("dimension", 6)
            self.left_hand: HandAdapter = DummyDexHand(hand_dimension)
            self.right_hand: HandAdapter = DummyDexHand(hand_dimension)

    def step(self) -> dict[str, np.ndarray] | None:
        qpos, hand_qpos, ee_pose, head_pose = self.observe()

        batch = {}
        frames = self.cam.grab()
        for key, cam in self.eval_cfg.cameras.items():
            try:
                batch[key] = frames[cam.name][cam.stream].astype(np.uint8)

                if rr:
                    img = frames[cam.name][cam.stream][:, 240:-240, :]
                    img = cv2.resize(img, (256, 256), interpolation=cv2.INTER_LINEAR).astype(np.uint8)
                    rr.log(f"/observation/{cam.name}/{cam.stream}", rr.Image(img))
            except KeyError as e:
                raise KeyError(f"Camera {cam.name} not found in frames. Available cameras: {frames.keys()}") from e

        if rr:
            rr.set_time_sequence("step", self._step)
            # rr.set_time_seconds("ts", time.time())
        self._step += 1

        obs_dict = {
            "left_leg": qpos[0:6],
            "right_leg": qpos[6:12],
            "waist": qpos[12:15],
            "neck": qpos[15:18],
            "left_arm": qpos[18:25],
            "right_arm": qpos[25:32],
            "left_hand": hand_qpos[0:6],
            "right_hand": hand_qpos[6:12],
            "left_ee_pose": ee_pose[0:9],
            "right_ee_pose": ee_pose[9:18],
            "head_pose": head_pose,
        }

        # assert set(obs_dict.keys()).issuperset(set(self.eval_cfg.states)), (
        #     f"Mismatch modality keys: {obs_dict.keys()} {self.eval_cfg.states}"
        # )
        # obs = np.concatenate([obs_dict[key] for key in self.eval_cfg.states])
        obs = self.policy.prepare_observation(obs_dict)

        if rr:
            rr.log(
                "/observation/state",
                rr.BarChart(np.concatenate([obs_dict[key] for key in self.eval_cfg.display_keys.states])),
            )

        batch.update(
            {
                "observation.state": obs,
                "task": [self.eval_cfg.prompt],
            }
        )
        # for k, v in batch.items():
        #     if k != "task":
        #         logger.debug(f"{k}: {v.shape}")

        for k, v in obs_dict.items():
            if k in self.eval_cfg.modality_mask.states:
                obs_dict[k] = np.zeros_like(v)

        action = self.policy.select_action(batch=batch)

        # array_action to dict action

        for k, v in action.items():
            if k in self.eval_cfg.modality_mask.actions:
                action[k] = np.zeros_like(v)

        if rr:
            rr.log("/action", rr.BarChart(np.concatenate([action[k] for k in self.eval_cfg.display_keys.actions])))

        return action

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

    def control_hands(self, hand_action):
        # left, right = self.hand_action_convert(left_qpos, right_qpos, filtering=True)

        filtered_hand_action = self.hand_filter.next(time.time(), hand_action)
        left = filtered_hand_action[:6]
        right = filtered_hand_action[6:]
        self.left_hand.set_positions(left)  # type: ignore
        self.right_hand.set_positions(right)  # type: ignore

        return np.hstack([left, right])

    def control_joints(self, qpos):
        if len(qpos) == 20:
            qpos = np.hstack(
                [
                    np.zeros(12),
                    qpos,
                ]
            )
        if len(qpos) != 32:
            raise ValueError("Invalid qpos shape.")
        qpos = self.joint_filter.next(time.time(), qpos)
        self.upsampler.put(qpos)
        return qpos

    def init_control_joints(self):
        if self._init_command_sent:
            return
        # self.client.init_command_joints(self.q_real)
        self.upsampler.start()
        self.upsampler.put(self.q_real)
        self._init_command_sent = True
        logger.info("Init command sent.")

    def pause_robot(self):
        logger.info("Pausing robot...")
        self.upsampler.pause()
        # self.client.move_joints(ControlGroup.ALL, self.client.joint_positions, gravity_compensation=False)

    def end(self):
        self.upsampler.stop()
        self.upsampler.join()
        self.client.disconnect()
        with ThreadPoolExecutor(max_workers=2) as executor:
            f = []
            f.append(executor.submit(self.left_hand.reset))
            f.append(executor.submit(self.left_hand.reset))

            for future in f:
                future.result()
        self.left_hand.stop()
        self.right_hand.stop()

        import os

        os._exit(0)


class PointCloudEvalRobot(EvalRobot):
    def __init__(self, cfg: DictConfig):
        super().__init__(cfg)

    def step(self):
        qpos, hand_qpos, ee_pose, head_pose = self.observe()

        frames = self.cam.grab()
        if frames["top"]["rgb"] is None:
            logger.warning("No top image.")
            return
        if rr:
            rr.set_time_sequence("step", self._step)
        # rr.set_time_seconds("ts", time.time())
        self._step += 1
        # if rr:
        #     rr.log("/observation/images/top", rr.Image(frames["top"]["rgb"].astype(np.uint8)))

        # TODO: read self.eval_cfg.cameras
        images_top = torch.tensor(frames["top"]["rgb"], dtype=torch.float32)
        # h, w, c to b, c, h, w
        images_top = images_top.expand(1, -1, -1, -1).permute(0, 3, 1, 2).to(self.device)

        depth = frames["top"]["depth"]

        pointcloud = None
        if depth is not None:
            if rr:
                rr.log("/observation/images/depth", rr.Image(depth.astype(np.uint8)))

            pointcloud = create_colored_point_cloud_from_depth_oak(depth * 1e-3, num_points=4096)
            pointcloud = torch.tensor(pointcloud, dtype=torch.float32).expand(1, -1, -1).reshape(1, -1).to(self.device)

        # TODO: add injectable obs_transform()
        obs = np.concatenate([qpos[12:], hand_qpos])

        if rr:
            rr.log("/observation/state", rr.BarChart(obs.tolist()))
        obs = torch.tensor(obs, dtype=torch.float32).unsqueeze(0).to(self.device)

        logger.debug(f"Observation: {qpos.shape}, {hand_qpos.shape} {frames['top']['rgb'].shape}  {obs.shape}")

        batch = {
            "observation.state": obs,
            "observation.images.top": images_top,
            "observation.pointcloud": pointcloud,
        }
        for k, v in batch.items():
            if k != "task":
                logger.debug(f"{k}: {v.shape}")
        action = self.policy.select_action(batch=batch)
        action = action.cpu().numpy().squeeze()
        logger.debug(action)

        if rr:
            rr.log("/action", rr.BarChart(action.tolist()))

        return action
