import logging

import numpy as np
from dex_retargeting.retargeting_config import RetargetingConfig
from fourier_robot_descriptions.fourier import PACKAGE_PATH
from omegaconf import DictConfig, OmegaConf

from teleoperation.utils import ASSET_DIR, remap

logger = logging.getLogger(__name__)


class HandRetarget:
    def __init__(self, cfg: DictConfig) -> None:
        assets_dir = ASSET_DIR
        RetargetingConfig.set_default_urdf_dir(assets_dir)
        cfg_left = OmegaConf.to_container(cfg["left"], resolve=True)
        cfg_right = OmegaConf.to_container(cfg["right"], resolve=True)

        # append urdf path
        cfg_left["urdf_path"] = (PACKAGE_PATH / cfg_left["urdf_path"]).with_suffix(".urdf").absolute()
        cfg_right["urdf_path"] = (PACKAGE_PATH / cfg_right["urdf_path"]).with_suffix(".urdf").absolute()

        left_retargeting_config = RetargetingConfig.from_dict(cfg=cfg_left)
        right_retargeting_config = RetargetingConfig.from_dict(cfg=cfg_right)
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()
        self.hand_type = cfg.type
        self.tip_indices = cfg.tip_indices
        self.cfg = cfg

    @property
    def left_joint_names(self):
        return self.left_retargeting.joint_names

    @property
    def right_joint_names(self):
        return self.right_retargeting.joint_names

    def retarget(self, left_landmarks: np.ndarray, right_landmarks: np.ndarray):
        left_input = left_landmarks[self.tip_indices]
        right_input = right_landmarks[self.tip_indices]

        if self.left_retargeting.optimizer.retargeting_type.lower() == "dexpilot":
            # for dexpilot, we need to calculate vector between each tip
            left_input_processed = []
            for i in range(len(self.tip_indices)):
                for j in range(i + 1, len(self.tip_indices)):
                    left_input_processed.append(left_input[j] - left_input[i])
            for i in range(len(self.tip_indices)):
                left_input_processed.append(left_input[i] - left_landmarks[0])
            left_input = np.array(left_input_processed)

        if self.right_retargeting.optimizer.retargeting_type.lower() == "dexpilot":
            # for dexpilot, we need to calculate vector between each tip
            right_input_processed = []
            for i in range(len(self.tip_indices)):
                for j in range(i + 1, len(self.tip_indices)):
                    right_input_processed.append(right_input[j] - right_input[i])

            for i in range(len(self.tip_indices)):
                right_input_processed.append(right_input[i] - right_landmarks[0])
            right_input = np.array(right_input_processed)

        left_qpos = self.left_retargeting.retarget(left_input)
        right_qpos = self.right_retargeting.retarget(right_input)

        return left_qpos, right_qpos

    def qpos_to_real(self, left_qpos, right_qpos):
        """Convert hand joint angles to real values passed to the hand SDK"""

        left_qpos_real = remap(
            left_qpos[self.cfg.actuated_indices],
            self.left_retargeting.joint_limits[:, 0],
            self.left_retargeting.joint_limits[:, 1],
            self.cfg.range_max,
            self.cfg.range_min,
        )

        right_qpos_real = remap(
            right_qpos[self.cfg.actuated_indices],
            self.right_retargeting.joint_limits[:, 0],
            self.right_retargeting.joint_limits[:, 1],
            self.cfg.range_max,
            self.cfg.range_min,
        )

        return left_qpos_real, right_qpos_real

    def actuated_qpos_to_real(self, left_qpos, right_qpos):
        """Convert hand joint angles to real values passed to the hand SDK"""

        left_qpos_real = remap(
            left_qpos,
            self.left_retargeting.joint_limits[:, 0],
            self.left_retargeting.joint_limits[:, 1],
            self.cfg.range_max,
            self.cfg.range_min,
        )

        right_qpos_real = remap(
            right_qpos,
            self.right_retargeting.joint_limits[:, 0],
            self.right_retargeting.joint_limits[:, 1],
            self.cfg.range_max,
            self.cfg.range_min,
        )

        return left_qpos_real, right_qpos_real

    def real_to_qpos(self, left_qpos_real, right_qpos_real):
        """Convert real values passed to the hand SDK to hand joint angles"""
        left_qpos = remap(
            left_qpos_real,
            self.cfg.range_max,
            self.cfg.range_min,
            self.left_retargeting.joint_limits[:, 0],
            self.left_retargeting.joint_limits[:, 1],
        )

        right_qpos = remap(
            right_qpos_real,
            self.cfg.range_max,
            self.cfg.range_min,
            self.right_retargeting.joint_limits[:, 0],
            self.right_retargeting.joint_limits[:, 1],
        )

        return left_qpos, right_qpos
