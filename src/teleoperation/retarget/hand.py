import numpy as np
from dex_retargeting.retargeting_config import RetargetingConfig
from omegaconf import DictConfig, OmegaConf

from teleoperation.utils import ASSET_DIR, remap


class HandRetarget:
    def __init__(self, cfg: DictConfig) -> None:
        assets_dir = ASSET_DIR
        RetargetingConfig.set_default_urdf_dir(assets_dir)
        left_retargeting_config = RetargetingConfig.from_dict(cfg=OmegaConf.to_container(cfg["left"], resolve=True))
        right_retargeting_config = RetargetingConfig.from_dict(cfg=OmegaConf.to_container(cfg["right"], resolve=True))
        self.left_retargeting = left_retargeting_config.build()
        self.right_retargeting = right_retargeting_config.build()
        self.hand_type = cfg.type
        self.tip_indices = cfg.tip_indices

    @property
    def left_joint_names(self):
        return self.left_retargeting.joint_names

    @property
    def right_joint_names(self):
        return self.right_retargeting.joint_names

    def retarget(self, left_landmarks: np.ndarray, right_landmarks: np.ndarray):
        left_qpos = self.left_retargeting.retarget(left_landmarks[self.tip_indices])
        right_qpos = self.right_retargeting.retarget(right_landmarks[self.tip_indices])
        return left_qpos, right_qpos

    def qpos_to_real(self, left_qpos, right_qpos):
        if self.hand_type == "inspire":
            left_qpos_real = remap(
                left_qpos[[4, 6, 2, 0, 9, 8]],
                self.left_retargeting.joint_limits[:, 0],
                self.left_retargeting.joint_limits[:, 1],
                1000,
                0,
            ).astype(int)

            right_qpos_real = remap(
                right_qpos[[4, 6, 2, 0, 9, 8]],
                self.right_retargeting.joint_limits[:, 0],
                self.right_retargeting.joint_limits[:, 1],
                1000,
                0,
            ).astype(int)

            return left_qpos_real, right_qpos_real
        elif self.hand_type == "fourier":
            eps = 1e-3
            left_qpos_real = remap(
                left_qpos[[0, 2, 6, 4, 9, 8]],
                self.left_retargeting.joint_limits[:, 0],
                self.left_retargeting.joint_limits[:, 1],
                [10.3, 10.3, 10.3, 10.3, eps, 10.3],
                [eps, eps, eps, eps, 10.3, eps],
            )
            right_qpos_real = remap(
                right_qpos[[0, 2, 6, 4, 9, 8]],
                self.right_retargeting.joint_limits[:, 0],
                self.right_retargeting.joint_limits[:, 1],
                [10.3, 10.3, 10.3, 10.3, eps, 10.3],
                [eps, eps, eps, eps, 10.3, eps],
            )
            return left_qpos_real, right_qpos_real

        else:
            raise NotImplementedError

    def real_to_qpos(self, left_qpos_real, right_qpos_real):
        if self.hand_type == "inspire":
            left_qpos = remap(
                left_qpos_real,
                1000,
                0,
                self.left_retargeting.joint_limits[:, 0],
                self.left_retargeting.joint_limits[:, 1],
            )

            right_qpos = remap(
                right_qpos_real,
                1000,
                0,
                self.right_retargeting.joint_limits[:, 0],
                self.right_retargeting.joint_limits[:, 1],
            )

            return left_qpos, right_qpos
        elif self.hand_type == "fourier":
            eps = 1e-3
            left_qpos = remap(
                left_qpos_real,
                [10.3, 10.3, 10.3, 10.3, eps, 10.3],
                [eps, eps, eps, eps, 10.3, eps],
                self.left_retargeting.joint_limits[:, 0],
                self.left_retargeting.joint_limits[:, 1],
            )
            right_qpos = remap(
                right_qpos_real,
                [10.3, 10.3, 10.3, 10.3, eps, 10.3],
                [eps, eps, eps, eps, 10.3, eps],
                self.right_retargeting.joint_limits[:, 0],
                self.right_retargeting.joint_limits[:, 1],
            )
            return left_qpos, right_qpos

        else:
            raise NotImplementedError
