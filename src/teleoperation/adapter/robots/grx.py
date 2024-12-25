import logging
import time

import numpy as np
from fourier_grx_client import ControlGroup, RobotClient

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
        return self.client.joint_positions.copy()

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
        return (self.client.joint_positions.copy(),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self._move_to_default()

    def _move_to_default(self):
        logger.info("Moving to default position...")
        self.client.move_joints(
            ControlGroup.UPPER,
            positions=[0, 0, np.pi / 2, 0, 0, 0, 0, 0, 0, -np.pi / 2, 0, 0, 0, 0],
            degrees=False,
            gravity_compensation=False,
            duration=0.5,
            blocking=True,
        )

        time.sleep(0.1)

        self.client.move_joints(
            ControlGroup.UPPER,
            positions=[0, 0, np.pi / 2, -np.pi / 2, 0, 0, 0, 0, 0, -np.pi / 2, -np.pi / 2, 0, 0, 0],
            degrees=False,
            gravity_compensation=False,
            duration=0.5,
            blocking=True,
        )

        time.sleep(0.1)

        self.client.move_joints(self.controlled_joint_indices, positions=self.default_qpos, degrees=False, duration=1.0)
