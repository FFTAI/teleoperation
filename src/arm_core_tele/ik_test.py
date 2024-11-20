from omegaconf import DictConfig, OmegaConf
import time
from typing import Literal

import numpy as np
import pink
import pinocchio as pin
import qpsolvers

from filters import LPRotationFilter, OneEuroFilter
from robot_wrapper import RobotWrapper
from pathlib import Path


class IK_Robot(RobotWrapper):
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

    def get_transform(self,to_frame,from_frame):
        transform = self.configuration.get_transform(to_frame,from_frame).np
        return transform


    def so3_to_ortho6d(self,so3):
        return so3[:, :2].transpose().reshape(-1)

    def se3_to_xyzortho6d(self,se3):
        """
        Convert SE(3) to continuous 6D rotation representation.
        """
        so3 = se3[:3, :3]
        xyz = se3[:3, 3]
        ortho6d = self.so3_to_ortho6d(so3)
        return np.concatenate([xyz, ortho6d])

    def get_joint_pos_by_ik(self, group_name, target, dt):
        if group_name not in {'left_arm', 'right_arm', 'dual_arm'}:
            raise ValueError(f"Invalid group: '{group_name}'. Expected one of {'left_arm', 'right_arm', 'dual_arm'}.")

        left_ee_pose = self.get_transform("left_end_effector_link", "base_link")
        right_ee_pose = self.get_transform("right_end_effector_link", "base_link")
        head_pose = self.get_transform("head_yaw_link", "base_link")
        if group_name == 'left_arm':
           left_ee_pose = target
        elif group_name == 'right_arm':
            right_ee_pose = target 
        self.solve(left_target=left_ee_pose,right_target=right_ee_pose,head_target=head_pose,dt=dt)

        if group_name == 'left_arm':
            return self.configuration.q[18:25]
        elif group_name == 'right_arm':
            return self.configuration.q[25:32]
        elif group_name == 'dual_arm':
            return self.configuration.q[18:32]  



if __name__ == "__main__":
    # Gets the path of the current script ---->  /PATH/arm_core_tele
    script_dir = Path(__file__).resolve().parent
    ik_config_path = script_dir / "gr1.yaml"
    config = OmegaConf.load(ik_config_path)
    ik_robot = IK_Robot(config)
    left_link="left_end_effector_link"
    right_link="right_end_effector_link"
    head_link = "head_yaw_link"
    base_link="base_link"

    # current_joint_rad = np.zeros(32)
    current_joint_rad = [-1.03914849e-02, -1.49521595e-02, 4.94671687e-02, -1.95637330e-01, 6.63140509e-03, 
                         7.35934824e-02, 1.99541207e-02, -2.23552570e-01, 7.65770078e-02, -3.27066667e-02, 
                         -8.87471437e-02, -2.15255097e-02, 8.55307132e-02, 0.00000000e+00, -1.12775509e-04, 
                         -1.55657217e-05, -6.27236586e-05, -2.47431326e-05, -6.21536896e-02, -7.06810579e-02,
                           5.93466535e-02, -1.45898831e+00, -1.42174050e-01, -1.10677600e-01, -6.21185414e-02,
                             -2.84087946e-05, 1.18369942e-04, -7.97414905e-05, 3.06814909e-05, -1.53407440e-04,
                               3.22251813e-04, 6.78654978e-05]
    ik_robot.q_real = current_joint_rad
    print("############################ test FK (Forward dynamics)... ############################")
    left_ee_pose = ik_robot.get_transform(left_link, base_link)
    print(f"left_ee_pose:\n{left_ee_pose}")
    right_ee_pose = ik_robot.get_transform(right_link, base_link)
    print(f"right_ee_pose:\n{right_ee_pose}")
    head_pose = ik_robot.get_transform(head_link, base_link)
    print(f"head_pose:\n{head_pose}")

    # TODO: test ik

