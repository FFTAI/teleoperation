import logging
import time
from pathlib import Path

import numpy as np
from fourier_grx_dds.gravity_compensation import GravityCompensator, Upsampler
from fourier_grx_dds.utils import ControlMode, GR1ControlGroup

logger = logging.getLogger(__name__)


class GR1Robot_DDS:
    def __init__(
        self,
        dds_cfg_path: Path,
        controlled_joint_indices: list,
        default_qpos: list,
        named_links: dict,
    ):
        self.client = GravityCompensator(dds_cfg_path, target_hz=120)
        self.controlled_joint_indices = controlled_joint_indices
        self.default_qpos = default_qpos
        self.named_links = named_links

        logger.info(f"Initializing {self.__class__.__name__}...")
        logger.info(f"cfg_path: {dds_cfg_path}")
        logger.info(f"Config: {self.default_qpos}")

    @property
    def joint_positions(self):
        return self.client.joint_positions

    def connect(
        self,
    ):
        logger.info(f"Connecting to {self.__class__.__name__}...")
        time.sleep(1.0)
        self.client.enable()
        time.sleep(1.0)
        left_arm_position = [
            -0.10834163755741072,
            -0.07329939774949822,
            0.06528929994794762,
            -1.4866168673727456,
            -0.15687147335078633,
            -0.13071683482883256,
            -0.17893611111972085,
        ]
        right_arm_position = [
            0.10834163755741072,
            -0.07329939774949822,
            0.06528929994794762,
            -1.4866168673727456,
            -0.15687147335078633,
            -0.13071683482883256,
            0.17893611111972085,
        ]
        arm_position = left_arm_position + right_arm_position
        self.client.move_joints(GR1ControlGroup.UPPER, arm_position, duration=2.0)
        logger.info(f"Connected to {self.__class__.__name__}.")

    def command_joints(self, positions, gravity_compensation=False):
        self.client.move_joints(GR1ControlGroup.ALL, positions, duration=0.0, gravity_compensation=gravity_compensation)

    # For safety use of interpolation move to the initial position
    def init_command_joints(self, positions):
        print("###################################################")
        print("Prepare to move the robot to the current position, please be safe")
        input("init_control_joints(press enter):")
        print("###################################################")
        self.client.move_joints(GR1ControlGroup.ALL, positions, duration=1.0)

    def stop_joints(self):
        self.command_joints(self.joint_positions, gravity_compensation=False)

    def observe(self):
        return (self.client.joint_positions.copy(),)

    def disconnect(self):
        logger.info(f"Disconnecting from {self.__class__.__name__}...")
        self.client.set_enable(False)
