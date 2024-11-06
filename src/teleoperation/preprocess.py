import logging

import numpy as np
from omegaconf import DictConfig, OmegaConf

from .constants import (
    grd_yup2grd_zup,
    hand2inspire,
)
from .utils import fast_mat_inv, mat_update

logger = logging.getLogger(__name__)


class VuerPreprocessor:
    def __init__(self, cfg: DictConfig):
        self.cfg = cfg
        self.vuer_head_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, -0.2], [0, 0, 0, 1]], dtype=float)
        self.vuer_right_wrist_mat = np.array(
            [[1, 0, 0, 0.3], [0, 1, 0, 0.7], [0, 0, 1, -0.2], [0, 0, 0, 1]], dtype=float
        )
        self.vuer_left_wrist_mat = np.array(
            [[1, 0, 0, -0.3], [0, 1, 0, 0.7], [0, 0, 1, -0.2], [0, 0, 0, 1]], dtype=float
        )

        self.offset = np.array([0.1, 0, -0.6], dtype=float)

        self.wrist2left = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
        self.wrist2right = np.array([[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)

        if cfg.hand_type == "inspire":
            self.hand2fingers_left = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
            self.hand2fingers_right = np.array([[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)
        elif cfg.hand_type == "fourier":
            self.hand2fingers_left = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
            self.hand2fingers_right = np.array([[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=float)

    def calibrate(self, robot, head_mat, left_wrist_translation, right_wrist_translation):
        q = robot.q0.copy()
        # TODO: config this to enable calibration for ddifferent robots
        calibration_pose: dict[str, float] = OmegaConf.to_container(robot.config.calibration_pose, resolve=True)  # type: ignore
        for joint, angle in calibration_pose.items():
            q[robot.get_idx_q_from_name(joint)] = angle

        robot_head_pose = robot.frame_placement(q, self.cfg.named_links.head_link)
        robot_left_ee_pose = robot.frame_placement(q, self.cfg.named_links.left_end_effector_link)
        robot_right_ee_pose = robot.frame_placement(q, self.cfg.named_links.right_end_effector_link)

        left_offset = robot_left_ee_pose.translation - left_wrist_translation
        right_offset = robot_right_ee_pose.translation - right_wrist_translation

        self.offset += np.mean([left_offset, right_offset], axis=0)
        self.offset[1] = 0.0
        # self.offset[0] = 0.1
        logger.info(f"Calibration result: {self.offset}")

    def process(self, tv):
        self.vuer_head_mat = mat_update(self.vuer_head_mat, tv.head_matrix.copy())
        self.vuer_right_wrist_mat = mat_update(self.vuer_right_wrist_mat, tv.right_wrist.copy())
        self.vuer_left_wrist_mat = mat_update(self.vuer_left_wrist_mat, tv.left_wrist.copy())

        # change of basis
        head_mat = grd_yup2grd_zup @ self.vuer_head_mat @ fast_mat_inv(grd_yup2grd_zup)
        right_wrist_mat = grd_yup2grd_zup @ self.vuer_right_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)
        left_wrist_mat = grd_yup2grd_zup @ self.vuer_left_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)

        rel_left_wrist_mat = left_wrist_mat @ self.wrist2left
        # rel_left_wrist_mat[0:3, 3] = rel_left_wrist_mat[0:3, 3] - head_mat[0:3, 3]
        # z_offset = max(head_mat[2, 3] - 0.6, 0)
        # rel_left_wrist_mat[2, 3] = rel_left_wrist_mat[2, 3] - head_mat[2, 3]
        # rel_left_wrist_mat[0:3, 3] += np.array([0.1, 0, -0.6])
        rel_left_wrist_mat[0:3, 3] += self.offset

        rel_right_wrist_mat = right_wrist_mat @ self.wrist2right  # wTr = wTh @ hTr
        # rel_right_wrist_mat[2, 3] = rel_right_wrist_mat[2, 3] - head_mat[2, 3]
        # rel_right_wrist_mat[0:3, 3] += np.array([0.1, 0, -0.6])
        # rel_right_wrist_mat[0:3, 3] = rel_right_wrist_mat[0:3, 3] - head_mat[0:3, 3]

        rel_right_wrist_mat[0:3, 3] += self.offset

        # homogeneous
        left_fingers = np.concatenate([tv.left_landmarks.copy().T, np.ones((1, tv.left_landmarks.shape[0]))])
        right_fingers = np.concatenate([tv.right_landmarks.copy().T, np.ones((1, tv.right_landmarks.shape[0]))])

        # left_fingers = tv.left_landmarks.copy()[:, 3, :].reshape(-1, 4).T
        # right_fingers = tv.right_landmarks.copy()[:, 3, :].reshape(-1, 4).T

        # change of basis
        left_fingers = grd_yup2grd_zup @ left_fingers
        right_fingers = grd_yup2grd_zup @ right_fingers

        rel_left_fingers = fast_mat_inv(left_wrist_mat) @ left_fingers
        rel_right_fingers = fast_mat_inv(right_wrist_mat) @ right_fingers
        rel_left_fingers = (self.hand2fingers_left.T @ rel_left_fingers)[0:3, :].T
        rel_right_fingers = (self.hand2fingers_right.T @ rel_right_fingers)[0:3, :].T

        # print("__________-")
        # print(self.vuer_right_wrist_mat[:3, 3])
        # print(right_wrist_mat[:3, 3])
        # print(rel_right_wrist_mat[:3, 3])

        return (
            head_mat,
            rel_left_wrist_mat,
            rel_right_wrist_mat,
            rel_left_fingers,
            rel_right_fingers,
        )

    def get_hand_gesture(self, tv):
        self.vuer_right_wrist_mat = mat_update(self.vuer_right_wrist_mat, tv.right_hand.copy())
        self.vuer_left_wrist_mat = mat_update(self.vuer_left_wrist_mat, tv.left_hand.copy())

        # change of basis
        right_wrist_mat = grd_yup2grd_zup @ self.vuer_right_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)
        left_wrist_mat = grd_yup2grd_zup @ self.vuer_left_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)

        left_fingers = np.concatenate([tv.left_landmarks.copy().T, np.ones((1, tv.left_landmarks.shape[0]))])
        right_fingers = np.concatenate([tv.right_landmarks.copy().T, np.ones((1, tv.right_landmarks.shape[0]))])

        # change of basis
        left_fingers = grd_yup2grd_zup @ left_fingers
        right_fingers = grd_yup2grd_zup @ right_fingers

        rel_left_fingers = fast_mat_inv(left_wrist_mat) @ left_fingers
        rel_right_fingers = fast_mat_inv(right_wrist_mat) @ right_fingers
        rel_left_fingers = (hand2inspire.T @ rel_left_fingers)[0:3, :].T
        rel_right_fingers = (hand2inspire.T @ rel_right_fingers)[0:3, :].T
        all_fingers = np.concatenate([rel_left_fingers, rel_right_fingers], axis=0)

        return all_fingers
