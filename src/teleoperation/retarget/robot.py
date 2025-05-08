from __future__ import annotations

import time
from typing import Literal

import numpy as np
import pink
import pinocchio as pin
import qpsolvers
from omegaconf import DictConfig

from teleoperation.filters import LPRotationFilter, OneEuroFilter
from teleoperation.retarget.hand import HandRetarget

from .robot_wrapper import RobotWrapper


class IKRobot(RobotWrapper):
    def __init__(self, config: DictConfig):
        super().__init__(config)

        self.tasks = {}
        self.barriers = {}

        self.left_positiion_filter = OneEuroFilter(
            min_cutoff=self.config.position_filter.min_cutoff,
            beta=self.config.position_filter.beta,
        )
        self.left_orientation_filter = LPRotationFilter(self.config.orientation_filter.alpha)
        self.right_positiion_filter = OneEuroFilter(
            min_cutoff=self.config.position_filter.min_cutoff,
            beta=self.config.position_filter.beta,
        )
        self.right_orientation_filter = LPRotationFilter(self.config.orientation_filter.alpha)

        if self.viz:
            from itertools import product

            import meshcat.geometry as g

            if config.debug:
                self.viz.viewer["left_ee_target"].set_object(g.Box([0.1, 0.1, 0.1]))
                self.viz.viewer["right_ee_target"].set_object(g.Box([0.1, 0.1, 0.1]))
                self.viz.viewer["head"].set_object(g.Box([0.1, 0.1, 0.1]))

            if config.get("debug_hand", False):
                for side, finger in product(["left", "right"], range(26)):
                    if finger == 0:
                        self.viz.viewer[f"{side}_hand/{finger}"].set_object(g.Box([0.02, 0.02, 0.02]))
                    else:
                        self.viz.viewer[f"{side}_hand/{finger}"].set_object(g.Sphere(0.01))
                    if side == "left":
                        self.viz.viewer[f"{side}_hand/{finger}"].set_property("color", [1, 0, 0, 1])
            self.viz.display(self.configuration.q)

        self.build_tasks()

    def filter_pose(self, pose: pin.SE3, side: Literal["left", "right"]):
        xyzquat = pin.SE3ToXYZQUAT(pose)
        t = time.time()
        if side == "left":
            xyzquat[:3] = self.left_positiion_filter.next(t, xyzquat[:3])
            xyzquat[3:] = self.left_orientation_filter.next(xyzquat[3:])
        else:
            xyzquat[:3] = self.right_positiion_filter.next(t, xyzquat[:3])
            xyzquat[3:] = self.right_orientation_filter.next(xyzquat[3:])

        return pin.XYZQUATToSE3(xyzquat)

    def build_tasks(self):
        r_hand_task = pink.tasks.RelativeFrameTask(
            self.config.named_links.right_end_effector_link,
            self.config.named_links.root_link,
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        l_hand_task = pink.tasks.RelativeFrameTask(
            self.config.named_links.left_end_effector_link,
            self.config.named_links.root_link,
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        head_task = pink.tasks.RelativeFrameTask(
            self.config.named_links.head_link,
            self.config.named_links.root_link,
            position_cost=0.0,
            orientation_cost=1.0,
            gain=0.5,
            lm_damping=1e-1,
        )

        posture_task = pink.tasks.PostureTask(cost=1e-2)
        self.tasks = {
            "r_hand_task": r_hand_task,
            "l_hand_task": l_hand_task,
            "head_task": head_task,
            "posture_task": posture_task,
        }

        if self.config.self_collision.enable:
            collision_barrier = pink.barriers.SelfCollisionBarrier(
                n_collision_pairs=len(self.robot.collision_model.collisionPairs),
                gain=20.0,
                safe_displacement_gain=1.0,
                d_min=self.config.self_collision.min_distance,
            )

            self.barriers = {
                "collision_barrier": collision_barrier,
            }

    def set_posture_target_from_current_configuration(self):
        self.tasks["posture_task"].set_target_from_configuration(self.configuration)

    def solve(
        self,
        left_target: np.ndarray,
        right_target: np.ndarray,
        head_target: np.ndarray | None,
        dt: float,
    ):
        # right_target = pin.XYZQUATToSE3(right_target)
        # left_target = pin.XYZQUATToSE3(left_target)

        right_target = pin.SE3(translation=right_target[:3, 3], rotation=right_target[:3, :3])
        left_target = pin.SE3(translation=left_target[:3, 3], rotation=left_target[:3, :3])

        left_target.translation = left_target.translation * self.config.body_scaling_factor
        right_target.translation = right_target.translation * self.config.body_scaling_factor

        left_target = self.filter_pose(left_target, "left")
        right_target = self.filter_pose(right_target, "right")

        if self.viz:
            self.viz.viewer["left_ee_target"].set_transform(left_target.homogeneous)
            self.viz.viewer["right_ee_target"].set_transform(right_target.homogeneous)

        self.tasks["r_hand_task"].set_target(right_target)
        self.tasks["l_hand_task"].set_target(left_target)

        if head_target is not None:
            head_target = pin.SE3(head_target[:3, :3], np.array([0.0, 0.0, 0.0]))
            self.tasks["head_task"].set_target(head_target)

        solver = qpsolvers.available_solvers[0]

        if "daqp" in qpsolvers.available_solvers:
            solver = "daqp"

        velocity = pink.solve_ik(
            self.configuration,
            self.tasks.values(),
            dt,
            solver=solver,
            barriers=self.barriers.values(),
            safety_break=False,
        )
        self.configuration.integrate_inplace(velocity, dt)


class DexRobot(IKRobot):
    def __init__(self, config: DictConfig):
        super().__init__(config.robot)

        self.left_hand_prefix = config.hand.prefix_left
        self.right_hand_prefix = config.hand.prefix_right
        self.hand_dimension = config.hand.dimension

        self.hand_filter = OneEuroFilter(min_cutoff=config.hand_filter.min_cutoff, beta=config.hand_filter.beta)
        self.joint_filter = OneEuroFilter(min_cutoff=config.joint_filter.min_cutoff, beta=config.joint_filter.beta)

        self.hand_retarget = HandRetarget(config.hand)

    def hand_action_convert(self, left_qpos, right_qpos, filtering=True) -> tuple[np.ndarray, np.ndarray]:
        left_qpos_real, right_qpos_real = self.hand_retarget.qpos_to_real(left_qpos, right_qpos)
        if not filtering:
            filtered_hand_qpos = np.hstack([left_qpos_real, right_qpos_real])
        else:
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

        return filtered_hand_qpos[: self.hand_dimension], filtered_hand_qpos[self.hand_dimension :]

    def set_hand_joints(self, left_hand_qpos: np.ndarray, right_hand_qpos: np.ndarray):
        """Set the joint positions of the hands to pinocchio

        Args:
            left_hand_qpos (ndarray): Hand qpos in radians
            right_hand_qpos (ndarray): Hand qpos in radians
        """
        # left_hand_qpos, right_hand_qpos = self.hand_retarget.real_to_qpos(left_hand_real, right_hand_real)
        if len(left_hand_qpos) == 6:
            left = np.zeros(11)
            left[0:2] = left_hand_qpos[0]
            left[2:4] = left_hand_qpos[1]
            left[4:6] = left_hand_qpos[3]
            left[6:8] = left_hand_qpos[2]
            left[8] = left_hand_qpos[5]
            left[9:11] = left_hand_qpos[4]
        else:
            left = left_hand_qpos

        if len(right_hand_qpos) == 6:
            right = np.zeros(11)
            right[0:2] = right_hand_qpos[0]
            right[2:4] = right_hand_qpos[1]
            right[4:6] = right_hand_qpos[3]
            right[6:8] = right_hand_qpos[2]
            right[8] = right_hand_qpos[5]
            right[9:11] = right_hand_qpos[4]
        else:
            right = right_hand_qpos

        # print(left, right, self.hand_retarget.left_joint_names)
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
