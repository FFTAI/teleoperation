import time
from typing import Literal

import numpy as np
import pink
import pinocchio as pin
import qpsolvers
from filters import LPRotationFilter, OneEuroFilter
from omegaconf import DictConfig

from .robot_wrapper import RobotWrapper


class Robot(RobotWrapper):
    def __init__(self, config: DictConfig):
        super().__init__(config.robot)

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
            import meshcat.geometry as g

            # self.viz.viewer["left_ee_target"].set_object(g.Box([0.1, 0.1, 0.1]))
            # self.viz.viewer["right_ee_target"].set_object(g.Box([0.1, 0.1, 0.1]))
            self.viz.viewer["head"].set_object(g.Box([0.1, 0.1, 0.1]))
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
        start_link = "base_link"
        r_hand_task = pink.tasks.RelativeFrameTask(
            "right_end_effector_link",
            start_link,
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        l_hand_task = pink.tasks.RelativeFrameTask(
            "left_end_effector_link",
            start_link,
            position_cost=50.0,
            orientation_cost=10.0,
            gain=0.7,
            lm_damping=1e-3,
        )

        head_task = pink.tasks.RelativeFrameTask(
            "head_yaw_link",
            "base_link",
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
        head_target: np.ndarray,
        dt: float,
    ):
        right_target = pin.XYZQUATToSE3(right_target)
        left_target = pin.XYZQUATToSE3(left_target)
        head_target = pin.SE3(head_target[:3, :3], np.array([0.0, 0.0, 0.0]))

        left_target.translation = left_target.translation * self.config.body_scaling_factor
        right_target.translation = right_target.translation * self.config.body_scaling_factor

        left_target = self.filter_pose(left_target, "left")
        right_target = self.filter_pose(right_target, "right")

        if self.viz:
            self.viz.viewer["left_ee_target"].set_transform(left_target.homogeneous)
            self.viz.viewer["right_ee_target"].set_transform(right_target.homogeneous)

        self.tasks["r_hand_task"].set_target(right_target)
        self.tasks["l_hand_task"].set_target(left_target)
        self.tasks["head_task"].set_target(head_target)

        solver = qpsolvers.available_solvers[0]

        if "quadprog" in qpsolvers.available_solvers:
            solver = "quadprog"

        velocity = pink.solve_ik(
            self.configuration,
            self.tasks.values(),
            dt,
            solver=solver,
            barriers=self.barriers.values(),
            safety_break=False,
        )
        self.configuration.integrate_inplace(velocity, dt)
