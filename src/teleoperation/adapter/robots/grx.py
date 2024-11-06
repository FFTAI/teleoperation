import logging
import time

from fourier_grx_client import ControlGroup, RobotClient
from omegaconf import DictConfig

from teleoperation.utils import se3_to_xyzortho6d

logger = logging.getLogger(__name__)


class GR1Robot:
    def __init__(
        self,
        namespace: str,
        controlled_joint_indices: list,
        default_qpos: list,
        named_links: dict,
    ):
        self.client = RobotClient(namespace=namespace)
        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links

        logger.info(f"Initializing {self.__class__.__name__}...")
        logger.info(f"Namespace: {namespace}")
        logger.info(f"Config: {self.default_qpos}")

    @property
    def joint_positions(self):
        return self.client.joint_positions

    def connect(
        self,
    ):
        logger.info(f"Connecting to {self.__class__.__name__}...")
        time.sleep(1.0)
        self.client.set_enable(True)
        time.sleep(1.0)
        # move to default position
        self._move_to_default()
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        self.client.move_joints(
            ControlGroup.ALL, positions=positions, degrees=False, gravity_compensation=gravity_compensation
        )

    def init_command_joints(self, positions):
        self.client.move_joints(
            ControlGroup.ALL,
            positions=positions,
            degrees=False,
            gravity_compensation=False,
            duration=0.5,
            blocking=True,
        )

    def stop_joints(self):
        self.command_joints(self.joint_positions, gravity_compensation=False)

    def observe(self):
        qpos = self.client.joint_positions
        left_pose, right_pose, head_pose = self._get_ee_pose()

        return qpos, left_pose, right_pose, head_pose

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self._move_to_default()

    def _move_to_default(self):
        self.client.move_joints(self.controlled_joint_indices, positions=self.default_qpos, degrees=False, duration=1.0)

    def _get_ee_pose(self):
        left_link = self.named_links["left_end_effector_link"]
        right_link = self.named_links["right_end_effector_link"]
        head_link = self.named_links["head_link"]
        root_link = self.named_links["root_link"]

        left_pose = self.client.get_transform(left_link, root_link)
        right_pose = self.client.get_transform(right_link, root_link)
        head_pose = self.client.get_transform(head_link, root_link)

        left_pose = se3_to_xyzortho6d(left_pose)
        right_pose = se3_to_xyzortho6d(right_pose)
        head_pose = se3_to_xyzortho6d(head_pose)

        return left_pose, right_pose, head_pose
