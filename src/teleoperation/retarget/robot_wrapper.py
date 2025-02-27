import itertools
import logging
from collections.abc import Sequence

import numpy as np
import pink
import pinocchio as pin
from omegaconf import DictConfig, OmegaConf

from teleoperation.utils import ASSET_DIR

logger = logging.getLogger(__name__)


class RobotWrapper:
    def __init__(self, config: DictConfig | str):
        if isinstance(config, str):
            self.config = OmegaConf.load(config)
        else:
            self.config = config
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=str(ASSET_DIR / self.config.urdf_path),
            package_dirs=[str(ASSET_DIR / d) for d in self.config.urdf_package_dirs],
            root_joint=pin.JointModelFreeFlyer(),
        )
        
        self.num_joints = self.config.num_joints

        if self.config.joints_to_lock:
            logger.info(f"Locking joints: {self.config.joints_to_lock}")
            self.robot = self.robot.buildReducedRobot(self.config.joints_to_lock)

        for name, limits in self.config.joint_limits.items():
            logger.info(f"Setting joint limits for {name}: {limits}")
            self.set_joint_limits(name, limits[0], limits[1])

        for name, limit in self.config.velocity_limits.items():
            logger.info(f"Setting velocity limits for {name}: {limit}")
            self.set_velocity_limit(name, limit)

        for frame in self.config.get("frames", []):
            logger.info(f"Adding dummy frame: {frame.name}")
            self.add_frame(frame.name, frame.parent)

        # self.robot.collision_model.addAllCollisionPairs()
        if self.config.self_collision.enable:
            for g1, g2 in itertools.combinations(self.config.self_collision.enabled_links, 2):
                logger.info(f"Adding collision pair: {g1} - {g2}")
                self.robot.collision_model.addCollisionPair(
                    pin.CollisionPair(
                        self.robot.collision_model.getGeometryId(g1 + "_0"),
                        self.robot.collision_model.getGeometryId(g2 + "_0"),
                    )
                )

        self.robot.rebuildData()

        self.robot.collision_data.enable_contact = True

        self.configuration = pink.Configuration(
            self.robot.model,
            self.robot.data,
            self.robot.q0,
            collision_model=self.robot.collision_model,
            collision_data=self.robot.collision_data,
        )
        self.configuration.update(self.robot.q0)

        self.viz = None
        if self.config.visualize:
            self.viz = pin.visualize.MeshcatVisualizer(
                self.robot.model, self.robot.collision_model, self.robot.visual_model
            )
            self.robot.setVisualizer(self.viz, init=False)
            self.viz.initViewer(open=False, loadModel=True)

            self.viz.displayCollisions(self.config.display_collisions)

            self.viz.displayFrames(
                True,
                [self.robot.model.getFrameId(f) for f in self.config.displayed_frames],
            )

            self.viz.display(self.configuration.q)

    @property
    def model(self):
        return self.robot.model

    @property
    def data(self):
        return self.robot.data

    @property
    def q0(self):
        return self.robot.q0

    @property
    def q_real(self):
        """Get q vector in the real robot convention (length is 32, order same as config.joint_names)."""
        q = []
        for name in self.config.joint_names:
            q.append(self.get_q_from_name(name))
        return np.array(q)

    @q_real.setter
    def q_real(self, q: np.ndarray):
        """Set from a q vector in the real robot convention (length is 32, order same as config.joint_names))."""
        self.set_joint_positions(self.config.joint_names, q)

    def q_real2pink(self, q: np.ndarray):
        """Convert q vector from real robot convention to pink convention."""
        q_pink = self.configuration.q.copy()
        for name, value in zip(self.config.joint_names, q, strict=True):
            if name in self.config.joints_to_lock:
                continue
            q_pink[self.get_idx_q_from_name(name)] = value
        return q_pink

    def q_pink2real(self, q: np.ndarray):
        """Convert q vector from pink convention to real robot convention."""
        q_real = []
        for name in self.config.joint_names:
            if name in self.config.joints_to_lock:
                q_real.append(0.0)
                continue
            q_real.append(q[self.get_idx_q_from_name(name)])
        return np.array(q_real)

    def get_joint_by_name(self, name: str):
        """Get joint object by its name."""
        try:
            joint_id = self.model.getJointId(name)
            joint = self.model.joints[joint_id]
            return joint
        except IndexError as err:
            raise IndexError(f"Joint {name} not found in robot model") from err

    def get_idx_q_from_name(self, name: str):
        """Get joint index in Pinocchio configuration vector by its name.

        Args:
            name (str): Name of the joint.
        """
        return self.get_joint_by_name(name).idx_q

    def get_q_from_name(self, name: str):
        """Get joint position in Pinocchio configuration vector by its name.

        Args:
            name (str): Name of the joint.
        """
        if name in self.config.joints_to_lock:
            return 0.0
        return self.configuration.q[self.get_idx_q_from_name(name)]

    def get_idx_v_from_name(self, name: str):
        """Get joint index in Pinocchio velocity vector by its name.

        Args:
            name (str): Name of the joint.
        """
        return self.get_joint_by_name(name).idx_v

    def get_v_from_name(self, name: str):
        """Get joint velocity in Pinocchio velocity vector by its name.

        Args:
            name (str): Name of the joint.
        """
        if name in self.config.joints_to_lock:
            return 0.0
        return self.configuration.data.dq_after[self.get_idx_v_from_name(name)]

    def add_frame(self, frame_name: str, parent: str, transform: pin.SE3 | None = None):
        """Add a dummy frame to the robot model.

        Args:
            frame_name (str): Name of the frame.
            parent (str): Name of the parent frame.
            transform (pin.SE3 | None, optional): Transform from parent frame to the new frame. Defaults to None. If None, identity transform is used.
        """
        if frame_name in self.model.names:
            logger.warning(f"Frame {frame_name} already exists in robot model")
            return
        if self.model.existFrame(parent) is not True:
            logger.warning(f"Parent frame {parent} not found in robot model")
            return

        if transform is None:
            transform = pin.SE3.Identity()
        self.model.addFrame(
            pin.Frame(
                frame_name,
                self.model.frames[self.model.getFrameId(parent)].parent,
                self.model.getFrameId(parent),
                transform,
                pin.FrameType.OP_FRAME,
            )
        )

    def set_joint_positions(
        self,
        joint_names: list[str],
        positions: np.ndarray,
        degrees: bool = False,
        clip: bool = True,
    ):
        """Set joint positions in the robot configuration.

        Args:
            joint_names (list[str]): Names of the joints.
            positions (np.ndarray): Joint positions, AKA `q` vector in the robot definition.
            degrees (bool, optional): If True, positions are in degrees. Defaults to False.
            clip (bool, optional): If True, clip joint limits. Defaults to True.
        """
        if degrees:
            positions = np.deg2rad(positions)
        current_q = self.configuration.q.copy()
        for joint_name, position in zip(joint_names, positions, strict=True):
            if joint_name in self.config.joints_to_lock:
                continue
            q_idx = self.get_idx_q_from_name(joint_name)

            # clip joint limits
            if clip:
                lower = self.robot.model.lowerPositionLimit[q_idx]
                upper = self.robot.model.upperPositionLimit[q_idx]
                current_q[q_idx] = np.clip(position, lower, upper)
            else:
                current_q[q_idx] = position
        self.configuration.update(current_q)

    def get_joint_positions(self, joint_names: Sequence[str]) -> np.ndarray:
        """Get joint positions of given joints.

        Args:
            joint_names (Sequence[str]): list of joint names

        Returns:
            q (NDArray): joint positions
        """
        return np.array([self.configuration.q[self.get_idx_q_from_name(name)] for name in joint_names])

    def set_joint_limits(self, joint_name: str, lower: float, upper: float):
        """Set joint upper and lower limits. Note this does not update the robot data.

        Args:
            joint_name (str): Name of the joint.
            lower (float): Lower limit.
            upper (float): Upper limit.
        """
        if joint_name not in self.robot.model.names:
            logger.warning(f"Joint {joint_name} not found in robot model")
            return
        self.robot.model.lowerPositionLimit[self.get_idx_q_from_name(joint_name)] = lower
        self.robot.model.upperPositionLimit[self.get_idx_q_from_name(joint_name)] = upper

    def set_velocity_limit(self, joint_name: str, limit: float):
        """Set joint velocity limit. Note this does not update the robot data.

        Args:
            joint_name (str): Name of the joint.
            limit (float): Velocity limit.
        """
        if joint_name not in self.robot.model.names:
            logger.warning(f"Joint {joint_name} not found in robot model")
            return
        self.robot.model.velocityLimit[self.get_idx_v_from_name(joint_name)] = limit

    def frame_placement(self, q: np.ndarray, frame_name: str, source_frame: str | None = None) -> pin.SE3:
        """Get the placement of a frame in the robot model.

        Args:
            q (np.ndarray): Joint positions.
            frame_name (str): Name of the frame.
            source_frame (str | None, optional): Name of the source frame. Defaults to None. If None, the default world frame is used.
        """
        if len(q) == self.num_joints:
            q = self.q_real2pink(q)
        elif len(q) != self.model.nq:
            raise ValueError(f"Invalid q vector length: {len(q)}, model num joints: {self.model.nq}; robot num joints: {self.num_joints}")

        frame_idx = self.model.getFrameId(frame_name)
        frame_transform = self.robot.framePlacement(q, frame_idx)
        if source_frame is None:
            return frame_transform
        source_frame_idx = self.model.getFrameId(source_frame)
        source_frame_transform = self.robot.framePlacement(q, source_frame_idx)
        return source_frame_transform.inverse() * frame_transform

    def get_transforms(self, frame_names: Sequence[str]) -> list[pin.SE3]:
        return [self.configuration.get_transform_frame_to_world(frame_name).np for frame_name in frame_names]

    def update_display(self):
        """Display the current configuration in the visualizer."""
        if self.viz:
            self.viz.display(self.configuration.q)
