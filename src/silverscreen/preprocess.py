import numpy as np

from .constants import (
    grd_yup2grd_zup,
    hand2fourier_left,
    hand2fourier_right,
    hand2inspire,
    hand2left,
    hand2right,
)
from .utils import fast_mat_inv, mat_update


class VuerPreprocessor:
    def __init__(self, hand_type="inspire"):
        self.vuer_head_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, -0.2], [0, 0, 0, 1]])
        self.vuer_right_wrist_mat = np.array([[1, 0, 0, 0.3], [0, 1, 0, 0.7], [0, 0, 1, -0.2], [0, 0, 0, 1]])
        self.vuer_left_wrist_mat = np.array([[1, 0, 0, -0.3], [0, 1, 0, 0.7], [0, 0, 1, -0.2], [0, 0, 0, 1]])

        self.offset = np.array([0.1, 0, -0.6])

        if hand_type == "inspire":
            self.hand2fingers_left = hand2inspire
            self.hand2fingers_right = hand2inspire
        elif hand_type == "fourier":
            self.hand2fingers_left = hand2fourier_left
            self.hand2fingers_right = hand2fourier_right

    def calibrate(self, robot, head_mat, left_wrist_pose, right_wrist_pose):
        q = robot.q0.copy()
        q[robot.get_idx_q_from_name("right_elbow_pitch_joint")] = -np.pi / 2
        q[robot.get_idx_q_from_name("left_elbow_pitch_joint")] = -np.pi / 2
        robot_head_pose = robot.frame_placement(q, "head_yaw_link")
        robot_left_ee_pose = robot.frame_placement(q, "left_end_effector_link")
        robot_right_ee_pose = robot.frame_placement(q, "right_end_effector_link")

        left_offset = robot_left_ee_pose.translation - left_wrist_pose[:3]
        right_offset = robot_right_ee_pose.translation - right_wrist_pose[:3]

        self.offset += np.mean([left_offset, right_offset], axis=0)
        self.offset[1] = 0.0
        self.offset[0] = 0.1
        print(self.offset)

    def process(self, tv):
        self.vuer_head_mat = mat_update(self.vuer_head_mat, tv.head_matrix.copy())
        self.vuer_right_wrist_mat = mat_update(self.vuer_right_wrist_mat, tv.right_hand.copy())
        self.vuer_left_wrist_mat = mat_update(self.vuer_left_wrist_mat, tv.left_hand.copy())

        # change of basis
        head_mat = grd_yup2grd_zup @ self.vuer_head_mat @ fast_mat_inv(grd_yup2grd_zup)
        right_wrist_mat = grd_yup2grd_zup @ self.vuer_right_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)
        left_wrist_mat = grd_yup2grd_zup @ self.vuer_left_wrist_mat @ fast_mat_inv(grd_yup2grd_zup)

        rel_left_wrist_mat = left_wrist_mat @ hand2left
        # rel_left_wrist_mat[0:3, 3] = rel_left_wrist_mat[0:3, 3] - head_mat[0:3, 3]
        # z_offset = max(head_mat[2, 3] - 0.6, 0)
        # rel_left_wrist_mat[2, 3] = rel_left_wrist_mat[2, 3] - head_mat[2, 3]
        # rel_left_wrist_mat[0:3, 3] += np.array([0.1, 0, -0.6])
        rel_left_wrist_mat[0:3, 3] += self.offset

        rel_right_wrist_mat = right_wrist_mat @ hand2right  # wTr = wTh @ hTr
        # rel_right_wrist_mat[2, 3] = rel_right_wrist_mat[2, 3] - head_mat[2, 3]
        # rel_right_wrist_mat[0:3, 3] += np.array([0.1, 0, -0.6])
        # rel_right_wrist_mat[0:3, 3] = rel_right_wrist_mat[0:3, 3] - head_mat[0:3, 3]

        rel_right_wrist_mat[0:3, 3] += self.offset

        # homogeneous
        left_fingers = np.concatenate([tv.left_landmarks.copy().T, np.ones((1, tv.left_landmarks.shape[0]))])
        right_fingers = np.concatenate([tv.right_landmarks.copy().T, np.ones((1, tv.right_landmarks.shape[0]))])

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
